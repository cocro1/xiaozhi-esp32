/*
 * Local TCP control server for BLE HID.
 *
 * The ESP32 joins the same Wi-Fi as the PC. A PC-side script connects to
 * <esp32-ip>:8080 and sends one JSON command per line, e.g.
 *   {"t":"text","v":"hello world"}
 *   {"t":"key","m":2,"c":4}          // press 'a' with Left-Shift (mod bitmask, c=HID usage)
 *   {"t":"tap"}                      // left mouse click
 *   {"t":"move","dx":10,"dy":-5}     // move mouse
 *   {"t":"swipe","dx":200,"dy":0}    // press, move, release
 *   {"t":"media","c":233,"p":1}      // consumer code (233=VOL_UP), p=press 1/0
 *   {"t":"phonemode","v":1}          // auto-type STT to phone on/off
 *   {"t":"advertise","on":1}         // restart BLE advertising
 *   {"t":"status"}                   // -> {"ok":true,"ready":..,"connected":..,"mode":..}
 * Each line is acknowledged with {"ok":true} or {"ok":false,"err":"..."}.
 *
 * SPDX-License-Identifier: MIT
 */
#include "ble_hid.h"

#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

#include <cJSON.h>

#include "esp_log.h"

static const char *TAG = "BLE_HID_TCP";
#define HID_TCP_PORT 8080
#define HID_TCP_RX_BUF 512

void ble_hid_tcp_start(void);

static void send_ack(int fd, bool ok, const char *err)
{
    char resp[160];
    int n;
    if (ok) {
        n = snprintf(resp, sizeof(resp), "{\"ok\":true}\n");
    } else {
        n = snprintf(resp, sizeof(resp), "{\"ok\":false,\"err\":\"%s\"}\n", err ? err : "");
    }
    send(fd, resp, n, 0);
}

static void handle_line(int fd, const char *line)
{
    cJSON *root = cJSON_Parse(line);
    if (!root) {
        send_ack(fd, false, "bad json");
        return;
    }

    cJSON *t = cJSON_GetObjectItem(root, "t");
    if (!cJSON_IsString(t)) {
        send_ack(fd, false, "missing t");
        cJSON_Delete(root);
        return;
    }
    const char *type = t->valuestring;
    bool ok = true;
    const char *err = NULL;

    if (strcmp(type, "text") == 0) {
        cJSON *v = cJSON_GetObjectItem(root, "v");
        if (cJSON_IsString(v)) {
            ble_hid_send_text(v->valuestring);
        } else {
            ok = false; err = "missing v";
        }
    } else if (strcmp(type, "key") == 0) {
        cJSON *m = cJSON_GetObjectItem(root, "m");
        cJSON *c = cJSON_GetObjectItem(root, "c");
        if (cJSON_IsNumber(c)) {
            uint8_t mod = (cJSON_IsNumber(m) && m->valueint) ? (uint8_t)m->valueint : 0;
            ble_hid_key_combo(mod, (uint8_t)c->valueint);
        } else {
            ok = false; err = "missing c";
        }
    } else if (strcmp(type, "keydown") == 0) {
        cJSON *m = cJSON_GetObjectItem(root, "m");
        cJSON *c = cJSON_GetObjectItem(root, "c");
        if (cJSON_IsNumber(c)) {
            uint8_t mod = (cJSON_IsNumber(m) && m->valueint) ? (uint8_t)m->valueint : 0;
            ble_hid_key_press(mod, (uint8_t)c->valueint);
        } else {
            ok = false; err = "missing c";
        }
    } else if (strcmp(type, "keyup") == 0) {
        ble_hid_key_release();
    } else if (strcmp(type, "tap") == 0) {
        cJSON *b = cJSON_GetObjectItem(root, "b");
        uint8_t btn = (cJSON_IsNumber(b) && b->valueint) ? (uint8_t)b->valueint : BLE_HID_MOUSE_LEFT;
        ble_hid_mouse_click(btn);
    } else if (strcmp(type, "move") == 0) {
        cJSON *dx = cJSON_GetObjectItem(root, "dx");
        cJSON *dy = cJSON_GetObjectItem(root, "dy");
        if (cJSON_IsNumber(dx) && cJSON_IsNumber(dy)) {
            ble_hid_mouse_move((int8_t)dx->valueint, (int8_t)dy->valueint);
        } else {
            ok = false; err = "missing dx/dy";
        }
    } else if (strcmp(type, "press") == 0) {
        cJSON *b = cJSON_GetObjectItem(root, "b");
        uint8_t btn = (cJSON_IsNumber(b) && b->valueint) ? (uint8_t)b->valueint : BLE_HID_MOUSE_LEFT;
        ble_hid_mouse_press(btn);
    } else if (strcmp(type, "release") == 0) {
        ble_hid_mouse_release();
    } else if (strcmp(type, "swipe") == 0) {
        cJSON *dx = cJSON_GetObjectItem(root, "dx");
        cJSON *dy = cJSON_GetObjectItem(root, "dy");
        if (cJSON_IsNumber(dx) && cJSON_IsNumber(dy)) {
            int steps = 10;
            int8_t sx = (int8_t)(dx->valueint / steps);
            int8_t sy = (int8_t)(dy->valueint / steps);
            ble_hid_mouse_press(BLE_HID_MOUSE_LEFT);
            for (int i = 0; i < steps; i++) {
                ble_hid_mouse_move(sx, sy);
                vTaskDelay(pdMS_TO_TICKS(8));
            }
            ble_hid_mouse_release();
        } else {
            ok = false; err = "missing dx/dy";
        }
    } else if (strcmp(type, "media") == 0) {
        cJSON *c = cJSON_GetObjectItem(root, "c");
        cJSON *p = cJSON_GetObjectItem(root, "p");
        if (cJSON_IsNumber(c)) {
            bool pressed = cJSON_IsNumber(p) ? (p->valueint != 0) : true;
            ble_hid_media((uint8_t)c->valueint, pressed);
        } else {
            ok = false; err = "missing c";
        }
    } else if (strcmp(type, "phonemode") == 0) {
        cJSON *v = cJSON_GetObjectItem(root, "v");
        ble_hid_set_phone_mode(cJSON_IsNumber(v) ? (v->valueint != 0) : true);
    } else if (strcmp(type, "advertise") == 0) {
        ble_hid_restart_adv();
    } else if (strcmp(type, "status") == 0) {
        char resp[160];
        int n = snprintf(resp, sizeof(resp),
                         "{\"ok\":true,\"ready\":%d,\"connected\":%d,\"mode\":%d}\n",
                         ble_hid_ready() ? 1 : 0,
                         ble_hid_connected() ? 1 : 0,
                         ble_hid_phone_mode() ? 1 : 0);
        send(fd, resp, n, 0);
        cJSON_Delete(root);
        return;
    } else {
        ok = false; err = "unknown t";
    }

    send_ack(fd, ok, err);
    cJSON_Delete(root);
}

static void hid_tcp_task(void *arg)
{
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(HID_TCP_PORT);
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "HID control server listening on TCP port %d", HID_TCP_PORT);

    while (1) {
        int client = accept(listen_fd, NULL, NULL);
        if (client < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        ESP_LOGI(TAG, "control client connected");
        char buf[HID_TCP_RX_BUF];
        char line[HID_TCP_RX_BUF];
        int li = 0;
        while (1) {
            ssize_t n = recv(client, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break; /* disconnect or error */
            for (ssize_t i = 0; i < n; i++) {
                char ch = buf[i];
                if (ch == '\n' || ch == '\r') {
                    if (li > 0) {
                        line[li] = '\0';
                        handle_line(client, line);
                        li = 0;
                    }
                } else if (li < (int)sizeof(line) - 1) {
                    line[li++] = ch;
                }
            }
        }
        ESP_LOGI(TAG, "control client disconnected");
        close(client);
    }
    /* not reached */
}

void ble_hid_tcp_start(void)
{
    static bool started = false;
    if (started) return;
    started = true;
    BaseType_t ok = xTaskCreate(hid_tcp_task, "hid_tcp", 4096, NULL, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create tcp task");
        started = false;
    }
}
