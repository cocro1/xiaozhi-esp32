/*
 * BLE HID (keyboard + mouse + consumer) for Xiaozhi ESP32-S3.
 * Lets the device act as a Bluetooth LE HID peripheral so a phone can pair
 * with it like a Bluetooth keyboard/mouse, enabling programmatic phone control
 * (typing, taps, swipes, media keys) driven from a PC over a local TCP socket.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef BLE_HID_H__
#define BLE_HID_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HID modifier key masks (bit positions used by the keyboard report). */
#define BLE_HID_MOD_LCTRL  (1 << 0)
#define BLE_HID_MOD_LSHIFT (1 << 1)
#define BLE_HID_MOD_LALT   (1 << 2)
#define BLE_HID_MOD_LGUI   (1 << 3)
#define BLE_HID_MOD_RCTRL  (1 << 4)
#define BLE_HID_MOD_RSHIFT (1 << 5)
#define BLE_HID_MOD_RALT   (1 << 6)
#define BLE_HID_MOD_RGUI   (1 << 7)

/* Mouse button bits (passed to ble_hid_mouse_press / click). */
#define BLE_HID_MOUSE_LEFT   1
#define BLE_HID_MOUSE_RIGHT  2
#define BLE_HID_MOUSE_MIDDLE 4

/*
 * Lifecycle. Call ble_hid_start() once WiFi is connected (Blufi has already
 * torn the BT controller down, so this re-initialises it for HID).
 * Idempotent: only initialises once.
 */
void ble_hid_start(void);

/* Tear everything down (e.g. before re-entering WiFi config mode). */
void ble_hid_stop(void);

/* State queries. */
bool ble_hid_ready(void);      /* BT+HID initialised and advertising */
bool ble_hid_connected(void);  /* a host (phone) is paired & connected */

/* Keyboard. */
void ble_hid_send_text(const char *utf8);            /* type printable ASCII (CJK skipped, logged) */
void ble_hid_key_press(uint8_t modifier, uint8_t keycode);
void ble_hid_key_release(void);                      /* release all keys */
void ble_hid_key_combo(uint8_t modifier, uint8_t keycode); /* press + release */

/* Mouse. */
void ble_hid_mouse_move(int8_t dx, int8_t dy);
void ble_hid_mouse_press(uint8_t buttons);
void ble_hid_mouse_release(void);
void ble_hid_mouse_click(uint8_t buttons);

/* Consumer / media keys (see hid_dev.h HID_CONSUMER_* codes). */
void ble_hid_media(uint8_t consumer_code, bool pressed);

/* Phone mode: when enabled, recognised STT text is auto-typed to the phone. */
void ble_hid_set_phone_mode(bool on);
bool ble_hid_phone_mode(void);

/* Restart BLE advertising (e.g. after a disconnect while not auto-restarting). */
void ble_hid_restart_adv(void);

/* Start the local TCP control server (called internally after HID init). */
void ble_hid_tcp_start(void);

/*
 * Map a printable ASCII char to a HID keyboard usage code + modifier.
 * Returns the keycode (0 if unsupported). *modifier is set to 0 or a
 * BLE_HID_MOD_* mask. Non-ASCII (e.g. CJK) is not representable and returns 0.
 */
uint8_t ble_hid_ascii_to_key(char c, uint8_t *modifier);

#ifdef __cplusplus
}
#endif

#endif /* BLE_HID_H__ */
