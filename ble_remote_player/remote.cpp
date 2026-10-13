#include "remote.h"

#include "app_config.h"
#include "playback.h"

#include <Arduino.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_hid_common.h>
#include <esp_hidh.h>
#include <esp_hidh_bluedroid.h>
#include <esp_hidh_gattc.h>
#include <string.h>
#include <strings.h>

namespace {

// GAP コールバックが見つけたリモコンを loop() に渡す。HID 接続はブロックする
// ため、コールバック内では開始しない。
struct RemoteState {
  esp_bd_addr_t   address;
  uint8_t         addressType      = 0;
  volatile bool   found            = false;
  esp_hidh_dev_t* device           = nullptr;
  bool            connectedBefore = false;
  uint32_t        wakePlaybackAt   = 0;
  bool            lastPressed      = false;
};

RemoteState remote;

void restartScan() {
  remote.found = false;
  esp_ble_gap_start_scanning(0);
}

bool inputReportIsPressed(const uint8_t* data, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    if (data[i] != 0) return true;
  }
  return false;
}

void printInputReport(const esp_hidh_event_data_t* event) {
  Serial.printf("report usage=%d id=%u len=%u:", event->input.usage,
                event->input.report_id, event->input.length);
  for (uint16_t i = 0; i < event->input.length; i++) {
    Serial.printf(" %02X", event->input.data[i]);
  }
  Serial.println();
}

// アドバタイズパケットから名前を取り出す。完全名がなければ短縮名を見る。
const char* advertisedName(
    esp_ble_gap_cb_param_t::ble_scan_result_evt_param* result,
    uint8_t* length) {
  uint16_t total = result->adv_data_len + result->scan_rsp_len;
  uint8_t* name = esp_ble_resolve_adv_data_by_type(
      result->ble_adv, total, ESP_BLE_AD_TYPE_NAME_CMPL, length);
  if (name == nullptr || *length == 0) {
    name = esp_ble_resolve_adv_data_by_type(
        result->ble_adv, total, ESP_BLE_AD_TYPE_NAME_SHORT, length);
  }
  return reinterpret_cast<const char*>(name);
}

void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      esp_ble_gap_start_scanning(0);  // 0 = 見つかるまで無制限
      break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      auto* result = &param->scan_rst;
      if (result->search_evt != ESP_GAP_SEARCH_INQ_RES_EVT || remote.found) {
        break;
      }

      // vyviyon はボタンを押した時だけ名前付きでアドバタイズする。
      uint8_t length = 0;
      const char* name = advertisedName(result, &length);
      if (name == nullptr) break;

      size_t expectedLength = strlen(REMOTE_NAME);
      if (length < expectedLength ||
          strncasecmp(name, REMOTE_NAME, expectedLength) != 0) {
        break;
      }

      memcpy(remote.address, result->bda, sizeof(esp_bd_addr_t));
      remote.addressType = result->ble_addr_type;
      remote.found       = true;
      esp_ble_gap_stop_scanning();
      Serial.printf("リモコン発見 RSSI %d\n", result->rssi);
      break;
    }

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

void hidhCallback(void* arg, esp_event_base_t base, int32_t id, void* data) {
  auto* event = static_cast<esp_hidh_event_data_t*>(data);

  switch (static_cast<esp_hidh_event_t>(id)) {
    case ESP_HIDH_OPEN_EVENT:
      if (event->open.status == ESP_OK) {
        Serial.printf("リモコン接続 %s\n",
                      esp_hidh_dev_name_get(event->open.dev));
        // 最初の接続は鳴らさず、一度切れた後の再接続だけを押下とみなす。
        if (remote.connectedBefore) {
          triggerPlayback();
          remote.wakePlaybackAt = millis();
          Serial.println("再接続を押下とみなして再生");
        }
        remote.connectedBefore = true;
      } else {
        Serial.println("リモコン接続失敗、再スキャン");
        remote.device = nullptr;
        restartScan();
      }
      break;

    case ESP_HIDH_INPUT_EVENT: {
      if (event->input.length < 1) break;

      bool pressed = inputReportIsPressed(event->input.data,
                                          event->input.length);
      printInputReport(event);

      if (pressed && !remote.lastPressed) {
        if (remote.wakePlaybackAt != 0 &&
            millis() - remote.wakePlaybackAt < WAKE_PRESS_GUARD_MS) {
          Serial.println("起こした押下は再生済みなので無視");
        } else {
          triggerPlayback();
          Serial.println("再生");
        }
      }
      remote.lastPressed = pressed;
      break;
    }

    case ESP_HIDH_CLOSE_EVENT:
      Serial.println("リモコン切断、再スキャン");
      esp_hidh_dev_free(event->close.dev);
      remote.device = nullptr;
      restartScan();
      break;

    default:
      break;
  }
}

}  // namespace

void startRemote() {
  // A2DP が Bluedroid を起動した後に呼ぶ。
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
  esp_err_t error = esp_hidh_init(&config);
  if (error != ESP_OK) {
    Serial.printf("esp_hidh_init 失敗 %d\n", error);
    return;
  }

  esp_ble_scan_params_t scan = {};
  scan.scan_type          = BLE_SCAN_TYPE_ACTIVE;
  scan.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
  scan.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
  // A2DP と無線を共用するため、50ms ごとに 5ms だけスキャンする。
  scan.scan_interval      = 0x50;
  scan.scan_window        = 0x08;
  scan.scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE;
  esp_ble_gap_set_scan_params(&scan);

  Serial.println("リモコンを探しています");
}

void updateRemote() {
  if (!remote.found || remote.device != nullptr) return;

  remote.device = esp_hidh_dev_open(remote.address, ESP_HID_TRANSPORT_BLE,
                                    remote.addressType);
  if (remote.device == nullptr) {
    Serial.println("接続要求に失敗、再スキャン");
    restartScan();
  }
}
