// BLE シャッターリモコンのボタンで WAV を Bluetooth スピーカーから鳴らす。
//
// ESP32 を2役で使う。
//   Classic Bluetooth : A2DP ソース。スピーカーへ音を送る。
//   BLE               : HID ホスト。リモコンのボタンを受け取る。
// 両方を有効にするため、コントローラを BTDM モードで起動する。

#include "BluetoothA2DPSource.h"
#include "sound.h"

#include <esp_avrc_api.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_hid_common.h>
#include <esp_hidh.h>
#include <esp_hidh_bluedroid.h>
#include <esp_hidh_gattc.h>

const char* SPEAKER_NAME = "C06";
const char* REMOTE_NAME  = "vyviyon";  // 前方一致、大文字小文字は無視する

// スピーカーが見つからなくてもリモコンは使えるようにする。
// 起動からこの時間を過ぎたら、A2DP 接続を待たずに BLE を開始する。
const uint32_t BLE_START_TIMEOUT_MS = 30000;

// AVRCP (音量などの制御) は A2DP より少し遅れて繋がる。接続直後に音量を
// 送っても届かないことがあるので、この時間だけ待ってから送る。
const uint32_t VOLUME_DELAY_MS = 2000;
const uint8_t  VOLUME          = 100;  // 0〜127。ESP32 側とスピーカー本体の両方に効く

// 待機中に流す「聞こえない信号」。C06 は完全な無音が続くとアンプをミュート
// し、音が来てから解除するまでに約 1 秒かかる。その間の音、つまり再生の頭が
// 消える。小型スピーカーがほぼ鳴らせない低周波を小さく流し、ミュートさせない。
// 振幅は実機で調整する。小さすぎると頭が消え、大きすぎると「ブーン」と聞こえる。
// ±数程度では ESP32 側の音量処理 (VOLUME 100 で約 0.41 倍) と A2DP の圧縮で
// 0 に丸められて効かない。
const uint32_t KEEPALIVE_HZ        = 30;
const int16_t  KEEPALIVE_AMPLITUDE = 100;  // 約 -50 dBFS。0 で無効 (完全な無音を送る)
static_assert(SOUND_SAMPLE_RATE % KEEPALIVE_HZ == 0,
              "1 周期がサンプル数で割り切れないと波形の継ぎ目でノイズが出る");

// vyviyon は 5 分押さないと自分から切断して眠る。長く眠った後は、起こした
// 押下のレポートを送らずに捨てる (AB Shutter3 系で既知の挙動。ESP32 側で接続を
// 速めても直らない)。眠ったリモコンはボタンを押さない限り起きないので、切れた
// リモコンが繋がり直したこと自体を押下とみなして鳴らす。短い眠りの後は起こした
// 押下も接続直後に届く (実測で約 0.3 秒後) ので、二重に鳴らさないよう、この
// 時間内に届いた押下は無視する。
const uint32_t WAKE_PRESS_GUARD_MS = 1000;

const int LED_PIN = 2;

BluetoothA2DPSource a2dp_source;

volatile bool     playing = false;
volatile uint32_t playPos = 0;

// GAP コールバックが見つけたリモコンを loop() に渡すための受け渡し領域。
// esp_hidh_dev_open() はブロックするので、コールバック内では呼ばない。
static esp_bd_addr_t   remoteAddr;
static uint8_t         remoteAddrType = 0;
static volatile bool   remoteFound    = false;
static esp_hidh_dev_t* remoteDev      = nullptr;

// リモコンが押されたことを loop() に伝える。接続処理はコールバック内では始めない。
static volatile bool   pressedFlag    = false;

static void triggerPlayback() {
  playPos     = 0;
  playing     = true;
  pressedFlag = true;
}

int32_t getFrames(Frame* frame, int32_t frameCount) {
  static const uint32_t period = SOUND_SAMPLE_RATE / KEEPALIVE_HZ;
  static uint32_t       phase  = 0;

  for (int32_t i = 0; i < frameCount; i++) {
    uint32_t pos = playPos;
    int16_t  v;
    if (playing && pos < SOUND_FRAMES) {
      v       = soundData[pos];
      playPos = pos + 1;
    } else {
      playing = false;
      v       = KEEPALIVE_AMPLITUDE * sinf(2.0f * PI * phase / period);
      phase   = (phase + 1) % period;
    }
    frame[i].channel1 = v;
    frame[i].channel2 = v;
  }
  return frameCount;
}

// アドバタイズパケットから名前を取り出す。完全名がなければ短縮名を見る。
static const char* advName(esp_ble_gap_cb_param_t::ble_scan_result_evt_param* r,
                           uint8_t* len) {
  uint16_t total = r->adv_data_len + r->scan_rsp_len;
  uint8_t* name = esp_ble_resolve_adv_data_by_type(
      r->ble_adv, total, ESP_BLE_AD_TYPE_NAME_CMPL, len);
  if (name == nullptr || *len == 0) {
    name = esp_ble_resolve_adv_data_by_type(
        r->ble_adv, total, ESP_BLE_AD_TYPE_NAME_SHORT, len);
  }
  return (const char*)name;
}

static void gapCallback(esp_gap_ble_cb_event_t event,
                        esp_ble_gap_cb_param_t* param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      esp_ble_gap_start_scanning(0);  // 0 = 見つかるまで無制限
      break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      auto* r = &param->scan_rst;
      if (r->search_evt != ESP_GAP_SEARCH_INQ_RES_EVT || remoteFound) break;

      // vyviyon は待機中のアドバタイズに名前を載せず、ボタンを押すと名前付きで
      // 出し直す。名前のないものは読み飛ばす。
      uint8_t len = 0;
      const char* name = advName(r, &len);
      if (name == nullptr) break;

      // 前方一致で判定する。アドバタイズ名の大文字小文字は機種により揺れる。
      size_t want = strlen(REMOTE_NAME);
      if (len < want || strncasecmp(name, REMOTE_NAME, want) != 0) break;

      memcpy(remoteAddr, r->bda, sizeof(esp_bd_addr_t));
      remoteAddrType = r->ble_addr_type;
      remoteFound    = true;
      esp_ble_gap_stop_scanning();
      Serial.printf("リモコン発見 RSSI %d\n", r->rssi);
      break;
    }

    // リモコンからのペアリング要求。IO なしの Just Works で受ける。
    case ESP_GAP_BLE_SEC_REQ_EVT:
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      Serial.printf("ペアリング %s\n",
                    param->ble_security.auth_cmpl.success ? "成功" : "失敗");
      break;

    default:
      break;
  }
}

static void hidhCallback(void* arg, esp_event_base_t base, int32_t id,
                         void* data) {
  // このコールバックの中だけで使う。HID のイベントは1つのタスクで順に処理される。
  static bool     remoteConnectedBefore = false;
  static uint32_t wakePlayAt            = 0;  // 0 = 再接続で鳴らしたことがない

  auto* p = (esp_hidh_event_data_t*)data;

  switch ((esp_hidh_event_t)id) {
    case ESP_HIDH_OPEN_EVENT:
      if (p->open.status == ESP_OK) {
        Serial.printf("リモコン接続 %s\n", esp_hidh_dev_name_get(p->open.dev));
        // 起動後最初の接続は、押さなくても起きていたリモコンかもしれないので
        // 鳴らさない。一度切れた後の接続だけを押下とみなす。
        if (remoteConnectedBefore) {
          triggerPlayback();
          wakePlayAt = millis();
          Serial.println("再接続を押下とみなして再生");
        }
        remoteConnectedBefore = true;
      } else {
        Serial.println("リモコン接続失敗、再スキャン");
        remoteDev   = nullptr;  // 戻さないと loop() が二度と接続しに行かない
        remoteFound = false;
        esp_ble_gap_start_scanning(0);
      }
      break;

    case ESP_HIDH_INPUT_EVENT: {
      if (p->input.length < 1) break;

      // シャッターリモコンはボタンが1個しかなく、押した時だけレポートを送る。
      // 機種と iOS/Android 切替によってキーボードとコンシューマコントロールの
      // どちらを名乗るか、どのキーコードを送るかが変わるため、usage やキーの
      // 値では判定しない。「全バイトがゼロでない」状態への立ち上がりを押下と
      // みなす。離した時は全ゼロのレポートが来る。
      bool pressed = false;
      for (uint16_t i = 0; i < p->input.length; i++) {
        if (p->input.data[i] != 0) {
          pressed = true;
          break;
        }
      }

      // レポートの中身をそのまま出す。想定と違う動きをしたときの唯一の手掛かり。
      Serial.printf("report usage=%d id=%u len=%u:", p->input.usage,
                    p->input.report_id, p->input.length);
      for (uint16_t i = 0; i < p->input.length; i++) {
        Serial.printf(" %02X", p->input.data[i]);
      }
      Serial.println();

      static bool lastPressed = false;
      if (pressed && !lastPressed) {
        if (wakePlayAt != 0 && millis() - wakePlayAt < WAKE_PRESS_GUARD_MS) {
          Serial.println("起こした押下は再生済みなので無視");
        } else {
          triggerPlayback();
          Serial.println("再生");
        }
      }
      lastPressed = pressed;
      break;
    }

    case ESP_HIDH_CLOSE_EVENT:
      Serial.println("リモコン切断、再スキャン");
      esp_hidh_dev_free(p->close.dev);
      remoteDev   = nullptr;
      remoteFound = false;
      esp_ble_gap_start_scanning(0);
      break;

    default:
      break;
  }
}

// BLE 側の初期化。A2DP が Bluedroid を起動済みである必要がある。
static void startBle() {
  esp_ble_gap_register_callback(gapCallback);
  esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);

  esp_ble_auth_req_t authReq = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t   ioCap   = ESP_IO_CAP_NONE;
  uint8_t keySize = 16;
  uint8_t initKey = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rspKey  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &authReq,
                                 sizeof(authReq));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &ioCap, sizeof(ioCap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &keySize,
                                 sizeof(keySize));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &initKey,
                                 sizeof(initKey));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rspKey,
                                 sizeof(rspKey));

  esp_hidh_config_t config = {};
  config.callback         = hidhCallback;
  config.event_stack_size = 4096;
  esp_err_t err = esp_hidh_init(&config);
  if (err != ESP_OK) {
    Serial.printf("esp_hidh_init 失敗 %d\n", err);
    return;
  }

  esp_ble_scan_params_t scan = {};
  scan.scan_type          = BLE_SCAN_TYPE_ACTIVE;
  scan.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
  scan.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
  // 50ms ごとに 5ms だけ聞く (占有率 1 割)。無線は A2DP と共用なので、
  // 聞く時間を長くすると音声パケットが欠けて「プッ」というノイズになる。
  // リモコンはボタンを押すと短い間隔でアドバタイズするので、これでも拾える。
  scan.scan_interval      = 0x50;
  scan.scan_window        = 0x08;
  scan.scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE;
  esp_ble_gap_set_scan_params(&scan);  // 完了イベントでスキャンを開始する

  Serial.println("リモコンを探しています");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Serial.printf("音源 %lu フレーム / 約 %.1f 秒\n",
                (unsigned long)SOUND_FRAMES,
                SOUND_FRAMES / (float)SOUND_SAMPLE_RATE);

  // 既定は Classic のみで BLE 用メモリが解放される。BTDM にして BLE を残す。
  a2dp_source.set_default_bt_mode(ESP_BT_MODE_BTDM);
  // 一度ペアリングしたスピーカーは、多くの機種で名前の探索に応答しなくなる。
  // 前回繋いだアドレス (NVS に保存済み) へ直接接続しに行く。既定では無効。
  // 再試行は既定で 1000 回 (10 秒間隔)。使い切ると名前の探索に切り替わる。
  a2dp_source.set_auto_reconnect(true);
  a2dp_source.start(SPEAKER_NAME, getFrames);
  Serial.println("スピーカーを探しています");
}

void loop() {
  static bool     speakerConnected = false;
  static uint32_t connectedAt      = 0;
  static bool     volumeSent       = false;
  static bool     bleStarted       = false;

  bool connected = a2dp_source.is_connected();
  if (connected != speakerConnected) {
    speakerConnected = connected;
    if (connected) {
      connectedAt = millis();
      volumeSent  = false;  // 再接続のたびに音量を送り直す
      Serial.println("スピーカー接続");
    } else {
      Serial.println("スピーカー切断");
    }
  }

  // 音量は2か所に効く。
  //   ESP32 側のデジタル音量 : set_volume() で設定する。ただしスピーカーから
  //     音量変更の通知が来ると、ライブラリがその値で上書きしてしまう。
  //   スピーカー本体の音量   : AVRCP の絶対音量コマンドで送る。
  // set_volume() は値が変わらないと AVRCP を送らずに戻るので、コマンドは
  // 直接送る。
  if (speakerConnected && !volumeSent &&
      millis() - connectedAt > VOLUME_DELAY_MS) {
    a2dp_source.set_volume(VOLUME);
    esp_avrc_ct_send_set_absolute_volume_cmd(0, VOLUME);
    volumeSent = true;
    Serial.printf("音量 %u\n", VOLUME);
  }

  // 押されたのにスピーカーが繋がっていなければ、すぐ接続しに行く。背景の
  // 自動再接続は最大 10 秒待ちなので、使いたい瞬間を逃さないため。再生
  // フラグは立ったままなので、繋がった時点で頭から鳴る。
  // ライブラリの reconnect() は背景の自動再接続を止めてしまうので使わない。
  if (pressedFlag) {
    pressedFlag = false;
    esp_bd_addr_t* last = a2dp_source.get_last_peer_address();
    static const esp_bd_addr_t none = {};
    if (!speakerConnected && memcmp(*last, none, sizeof(none)) != 0) {
      Serial.println("スピーカー未接続、接続を試みる");
      a2dp_source.connect_to(*last);
    }
  }

  // 無線は1系統しかない。Classic の探索と BLE スキャンが競合しないよう、
  // スピーカーへの接続が終わってから BLE を開始する。ただしスピーカーが
  // 見つからないままリモコンも使えなくなるのは困るので、一定時間で見切る。
  if (!bleStarted && (speakerConnected || millis() > BLE_START_TIMEOUT_MS)) {
    if (!speakerConnected) Serial.println("スピーカー未接続のまま BLE を開始する");
    startBle();
    bleStarted = true;
  }

  if (remoteFound && remoteDev == nullptr) {
    remoteDev = esp_hidh_dev_open(remoteAddr, ESP_HID_TRANSPORT_BLE,
                                 remoteAddrType);
    if (remoteDev == nullptr) {
      Serial.println("接続要求に失敗、再スキャン");
      remoteFound = false;
      esp_ble_gap_start_scanning(0);
    }
  }

  digitalWrite(LED_PIN, playing ? HIGH : LOW);
  delay(10);
}
