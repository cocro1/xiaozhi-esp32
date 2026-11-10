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
 *   {"t":"status"}                   // -> {"ok":true,"ready":..,"connected":..,"mode":..,
 *                                   //     "abs":1,"fw":"abs-touch-v1","max":32767,..}
 *
 * Absolute Touch Screen commands (x/y are HID units 0..32767 unless *px*):
 *   {"t":"abscfg","w":1080,"h":2412} // set logical screen size for px commands
 *   {"t":"absdown","x":16000,"y":8000}
 *   {"t":"absup"}
 *   {"t":"abspos","x":16000,"y":8000}          // move while touching (drag frame)
 *   {"t":"abstap","x":16000,"y":8000,"ms":45}  // full click
 *   {"t":"absswipe","x1":..,"y1":..,"x2":..,"y2":..,"ms":300,"steps":12}
 *   {"t":"abspxtap","x":915,"y":279,"ms":45}   // same but accepts screen pixels
 *   {"t":"abspxswipe","x1":..,"y1":..,"x2":..,"y2":..,"ms":300,"steps":12}
 * Legacy move/press/release/tap/swipe remain available as compatibility shims.
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

static uint16_t clamp_hid(int v)
{
    if (v < 0) return 0;
    if (v > BLE_HID_ABS_MAX) return BLE_HID_ABS_MAX;
    return (uint16_t)v;
}

static bool get_xy(cJSON *root, const char *kx, const char *ky,
                   uint16_t *x, uint16_t *y)
{
    cJSON *jx = cJSON_GetObjectItem(root, kx);
    cJSON *jy = cJSON_GetObjectItem(root, ky);
    if (!cJSON_IsNumber(jx) || !cJSON_IsNumber(jy)) return false;
    *x = clamp_hid(jx->valueint);
    *y = clamp_hid(jy->valueint);
    return true;
}

static int jnum_or(cJSON *root, const char *k, int dflt)
{
    cJSON *j = cJSON_GetObjectItem(root, k);
    return cJSON_IsNumber(j) ? j->valueint : dflt;
}

static bool get_px(cJSON *root, const char *kx, const char *ky, int *x, int *y)
{
    cJSON *jx = cJSON_GetObjectItem(root, kx);
    cJSON *jy = cJSON_GetObjectItem(root, ky);
    if (!cJSON_IsNumber(jx) || !cJSON_IsNumber(jy)) return false;
    *x = jx->valueint;
    *y = jy->valueint;
    return true;
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
        /* Legacy: interpret dx/dy as screen pixels and perform a real
         * absolute touch gesture from the screen centre. The old
         * int8 sx=dx/10 trick overflowed and is no longer needed. */
        cJSON *dx = cJSON_GetObjectItem(root, "dx");
        cJSON *dy = cJSON_GetObjectItem(root, "dy");
        if (cJSON_IsNumber(dx) && cJSON_IsNumber(dy)) {
            int cx = ble_hid_screen_w() / 2;
            int cy = ble_hid_screen_h() / 2;
            ble_hid_touch_swipe(ble_hid_px_to_hid_x(cx),
                                ble_hid_px_to_hid_y(cy),
                                ble_hid_px_to_hid_x(cx + dx->valueint),
                                ble_hid_px_to_hid_y(cy + dy->valueint),
                                300, 12);
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
    /* ---- Absolute Touch Screen commands ------------------------------- */
    } else if (strcmp(type, "abscfg") == 0) {
        ble_hid_set_screen((uint16_t)jnum_or(root, "w", ble_hid_screen_w()),
                           (uint16_t)jnum_or(root, "h", ble_hid_screen_h()));
    } else if (strcmp(type, "absdown") == 0) {
        uint16_t x, y;
        if (get_xy(root, "x", "y", &x, &y)) {
            ble_hid_touch_down(x, y);
        } else { ok = false; err = "missing x/y"; }
    } else if (strcmp(type, "absup") == 0) {
        ble_hid_touch_up();
    } else if (strcmp(type, "abspos") == 0) {
        uint16_t x, y;
        if (get_xy(root, "x", "y", &x, &y)) {
            /* "d" overrides Tip Switch; defaults to down (drag frame). */
            ble_hid_touch_frame(x, y, jnum_or(root, "d", 1) != 0, true);
        } else { ok = false; err = "missing x/y"; }
    } else if (strcmp(type, "abstap") == 0) {
        uint16_t x, y;
        if (get_xy(root, "x", "y", &x, &y)) {
            ble_hid_touch_tap(x, y, (uint32_t)jnum_or(root, "ms", 45));
        } else { ok = false; err = "missing x/y"; }
    } else if (strcmp(type, "absswipe") == 0) {
        uint16_t x1, y1, x2, y2;
        if (get_xy(root, "x1", "y1", &x1, &y1) &&
            get_xy(root, "x2", "y2", &x2, &y2)) {
            ble_hid_touch_swipe(x1, y1, x2, y2,
                                (uint32_t)jnum_or(root, "ms", 300),
                                (uint16_t)jnum_or(root, "steps", 12));
        } else { ok = false; err = "missing x1/y1/x2/y2"; }
    } else if (strcmp(type, "abspxtap") == 0) {
        int px, py;
        if (get_px(root, "x", "y", &px, &py)) {
            ble_hid_touch_tap(ble_hid_px_to_hid_x(px),
                              ble_hid_px_to_hid_y(py),
                              (uint32_t)jnum_or(root, "ms", 45));
        } else { ok = false; err = "missing x/y"; }
    } else if (strcmp(type, "abspxswipe") == 0) {
        int x1, y1, x2, y2;
        if (get_px(root, "x1", "y1", &x1, &y1) &&
            get_px(root, "x2", "y2", &x2, &y2)) {
            ble_hid_touch_swipe(ble_hid_px_to_hid_x(x1), ble_hid_px_to_hid_y(y1),
                                ble_hid_px_to_hid_x(x2), ble_hid_px_to_hid_y(y2),
                                (uint32_t)jnum_or(root, "ms", 300),
                                (uint16_t)jnum_or(root, "steps", 12));
        } else { ok = false; err = "missing x1/y1/x2/y2"; }
    } else if (strcmp(type, "status") == 0) {
        char resp[200];
        int n = snprintf(resp, sizeof(resp),
                         "{\"ok\":true,\"ready\":%d,\"connected\":%d,\"mode\":%d,"
                         "\"abs\":1,\"fw\":\"%s\",\"max\":%d,"
                         "\"w\":%u,\"h\":%u}\n",
                         ble_hid_ready() ? 1 : 0,
                         ble_hid_connected() ? 1 : 0,
                         ble_hid_phone_mode() ? 1 : 0,
                         BLE_HID_FW_TAG, BLE_HID_ABS_MAX,
                         (unsigned)ble_hid_screen_w(),
                         (unsigned)ble_hid_screen_h());
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
