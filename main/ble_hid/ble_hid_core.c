/*
 * BLE HID core: initialisation, GAP/HID callbacks, and the public send API.
 *
 * The Xiaozhi firmware brings BT up only for BluFi provisioning and calls
 * Blufi::deinit() (which shuts the whole BT controller down) once WiFi connects.
 * So we re-initialise the controller + Bluedroid + HID profile *after* the
 * network is up. The board triggers ble_hid_start() from its network callback.
 *
 * SPDX-License-Identifier: MIT
 */
#include "ble_hid.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_defs.h"

#include "esp_hidd_prf_api.h"
#include "hid_dev.h"

static const char *TAG = "BLE_HID";

/* Friendly name shown when the phone scans for Bluetooth devices. */
#define HID_DEVICE_NAME "Xiaozhi-KM"

typedef enum {
    BLE_HID_UNINIT = 0,
    BLE_HID_INITIALIZING,
    BLE_HID_READY,
} ble_hid_state_t;

static uint16_t        s_conn_id = 0;
static bool            s_sec_conn = false;     /* host paired + encrypted */
static bool            s_adv_started = false;
static ble_hid_state_t s_state = BLE_HID_UNINIT;
static bool            s_phone_mode = false;
static TaskHandle_t    s_init_task = NULL;

/* ---- Advertising data / params (from ESP-IDF ble_hid_device_demo) -------- */
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = ESP_BLE_GAP_CONN_ITVL_MS(7.5),
    .max_interval = ESP_BLE_GAP_CONN_ITVL_MS(20),
    .appearance = 0x03c0,        /* HID Generic */
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = ESP_BLE_GAP_ADV_ITVL_MS(20),
    .adv_int_max = ESP_BLE_GAP_ADV_ITVL_MS(30),
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ---- GAP + HID callbacks -------------------------------------------------- */
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
    case ESP_HIDD_EVENT_REG_FINISH:
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            esp_ble_gap_set_device_name(HID_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT:
        s_conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "BLE host connected (conn_id=%d)", s_conn_id);
        break;
    case ESP_HIDD_EVENT_BLE_DISCONNECT:
        s_sec_conn = false;
        s_conn_id = 0;
        ESP_LOGI(TAG, "BLE host disconnected, restarting advertising");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
        ESP_LOGD(TAG, "LED report write");
        break;
    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        s_adv_started = true;
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            s_sec_conn = true;
            ESP_LOGI(TAG, "Pairing complete, secure link established");
        } else {
            ESP_LOGE(TAG, "Pairing failed, reason 0x%x",
                     param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

/* ---- Init task: bring BT controller + Bluedroid + HID profile back up ---- */
static void ble_hid_init_task(void *arg)
{
    esp_err_t ret;

    /* Free classic-BT memory (no-op / error if already released). */
    (void)esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "controller init failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "controller enable failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    esp_bluedroid_config_t blud_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&blud_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    if (esp_hidd_profile_init() != ESP_OK) {
        ESP_LOGE(TAG, "hidd profile init failed");
        goto fail;
    }
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* Security: bond with the phone, no IO capability on our side. */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;
    uint8_t key_size  = 16;
    uint8_t init_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key   = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  sizeof(uint8_t));

    s_state = BLE_HID_READY;
    s_init_task = NULL;
    ESP_LOGI(TAG, "BLE HID ready - pair with \"%s\" from your phone", HID_DEVICE_NAME);

    /* Bring up the local TCP control server. */
    ble_hid_tcp_start();
    vTaskDelete(NULL);
    return;

fail:
    ESP_LOGE(TAG, "BLE HID init failed; will retry on next (re)connection");
    s_state = BLE_HID_UNINIT;
    s_init_task = NULL;
    vTaskDelete(NULL);
}

void ble_hid_start(void)
{
    if (s_state != BLE_HID_UNINIT) {
        return; /* already initialised or in progress */
    }
    s_state = BLE_HID_INITIALIZING;
    BaseType_t ok = xTaskCreate(ble_hid_init_task, "ble_hid_init", 4096, NULL, 5, &s_init_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create init task");
        s_state = BLE_HID_UNINIT;
    }
}

void ble_hid_stop(void)
{
    if (s_state == BLE_HID_UNINIT) {
        return;
    }
    ESP_LOGI(TAG, "stopping BLE HID");
    esp_hidd_profile_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    s_state = BLE_HID_UNINIT;
    s_sec_conn = false;
    s_adv_started = false;
    s_conn_id = 0;
}

bool ble_hid_ready(void)     { return s_state == BLE_HID_READY && s_adv_started; }
bool ble_hid_connected(void) { return s_sec_conn; }

void ble_hid_set_phone_mode(bool on) { s_phone_mode = on; }
bool ble_hid_phone_mode(void)        { return s_phone_mode; }

void ble_hid_restart_adv(void)
{
    if (s_state == BLE_HID_READY) {
        esp_ble_gap_start_advertising(&hidd_adv_params);
    }
}

/* ---- Send API ------------------------------------------------------------ */
void ble_hid_key_press(uint8_t modifier, uint8_t keycode)
{
    if (!s_sec_conn) return;
    uint8_t k[1] = { keycode };
    esp_hidd_send_keyboard_value(s_conn_id, modifier, k, 1);
}

void ble_hid_key_release(void)
{
    if (!s_sec_conn) return;
    esp_hidd_send_keyboard_value(s_conn_id, 0, NULL, 0);
}

void ble_hid_key_combo(uint8_t modifier, uint8_t keycode)
{
    if (!s_sec_conn) return;
    ble_hid_key_press(modifier, keycode);
    vTaskDelay(pdMS_TO_TICKS(30));
    ble_hid_key_release();
    vTaskDelay(pdMS_TO_TICKS(30));
}

void ble_hid_send_text(const char *utf8)
{
    if (!s_sec_conn || !utf8) return;
    for (const char *p = utf8; *p; ++p) {
        char c = *p;
        if ((uint8_t)c > 127) {
            ESP_LOGW(TAG, "skip non-ASCII 0x%02x (CJK not representable via HID keyboard)",
                     (uint8_t)c);
            continue;
        }
        uint8_t mod = 0, code = 0;
        if (ble_hid_ascii_to_key(c, &mod, &code) == 0) {
            ESP_LOGW(TAG, "skip unsupported char '%c'", c);
            continue;
        }
        uint8_t k[1] = { code };
        esp_hidd_send_keyboard_value(s_conn_id, mod, k, 1);
        vTaskDelay(pdMS_TO_TICKS(25));
        esp_hidd_send_keyboard_value(s_conn_id, 0, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void ble_hid_mouse_move(int8_t dx, int8_t dy)
{
    if (!s_sec_conn) return;
    esp_hidd_send_mouse_value(s_conn_id, 0, dx, dy);
}

void ble_hid_mouse_press(uint8_t buttons)
{
    if (!s_sec_conn) return;
    esp_hidd_send_mouse_value(s_conn_id, buttons, 0, 0);
}

void ble_hid_mouse_release(void)
{
    if (!s_sec_conn) return;
    esp_hidd_send_mouse_value(s_conn_id, 0, 0, 0);
}

void ble_hid_mouse_click(uint8_t buttons)
{
    if (!s_sec_conn) return;
    esp_hidd_send_mouse_value(s_conn_id, buttons, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_hidd_send_mouse_value(s_conn_id, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void ble_hid_media(uint8_t consumer_code, bool pressed)
{
    if (!s_sec_conn) return;
    esp_hidd_send_consumer_value(s_conn_id, consumer_code, pressed);
}

/* Map a printable ASCII char to a HID keyboard usage code + modifier. */
uint8_t ble_hid_ascii_to_key(char c, uint8_t *modifier)
{
    *modifier = 0;
    if (c >= 'a' && c <= 'z') return 0x04 + (c - 'a');
    if (c >= 'A' && c <= 'Z') { *modifier = BLE_HID_MOD_LSHIFT; return 0x04 + (c - 'A'); }
    if (c >= '0' && c <= '9') return 0x1E + (c - '0');
    switch (c) {
    case ' ':  return 0x2C;
    case '\n': case '\r': return 0x28;
    case '\t': return 0x2B;
    case '-': case '_': if (c == '_') *modifier = BLE_HID_MOD_LSHIFT; return 0x2D;
    case '=': case '+': if (c == '+') *modifier = BLE_HID_MOD_LSHIFT; return 0x2E;
    case '[': case '{': if (c == '{') *modifier = BLE_HID_MOD_LSHIFT; return 0x2F;
    case ']': case '}': if (c == '}') *modifier = BLE_HID_MOD_LSHIFT; return 0x30;
    case '\\': case '|': if (c == '|') *modifier = BLE_HID_MOD_LSHIFT; return 0x31;
    case ';': case ':': if (c == ':') *modifier = BLE_HID_MOD_LSHIFT; return 0x33;
    case '\'': case '"': if (c == '"') *modifier = BLE_HID_MOD_LSHIFT; return 0x34;
    case '`': case '~': if (c == '~') *modifier = BLE_HID_MOD_LSHIFT; return 0x35;
    case ',': case '<': if (c == '<') *modifier = BLE_HID_MOD_LSHIFT; return 0x36;
    case '.': case '>': if (c == '>') *modifier = BLE_HID_MOD_LSHIFT; return 0x37;
    case '/': case '?': if (c == '?') *modifier = BLE_HID_MOD_LSHIFT; return 0x38;
    case '!': *modifier = BLE_HID_MOD_LSHIFT; return 0x1E;
    case '@': *modifier = BLE_HID_MOD_LSHIFT; return 0x1F;
    case '#': *modifier = BLE_HID_MOD_LSHIFT; return 0x20;
    case '$': *modifier = BLE_HID_MOD_LSHIFT; return 0x21;
    case '%': *modifier = BLE_HID_MOD_LSHIFT; return 0x22;
    case '^': *modifier = BLE_HID_MOD_LSHIFT; return 0x23;
    case '&': *modifier = BLE_HID_MOD_LSHIFT; return 0x24;
    case '*': *modifier = BLE_HID_MOD_LSHIFT; return 0x25;
    case '(': *modifier = BLE_HID_MOD_LSHIFT; return 0x26;
    case ')': *modifier = BLE_HID_MOD_LSHIFT; return 0x27;
    default:  return 0;
    }
}
