# yuPad43 ZMK driver

[日本語](README.md) | English

A ZMK module for the IQS9151-based yuPad43 touchpad (10 RX × 9 TX), targeting ZMK v0.3.0.

[Connection guide](https://github.com/yuchamichami/yuPad43/blob/main/README.en.md) · [ZMK setup](https://github.com/yuchamichami/yuPad43/blob/main/docs/zmk.en.md) · [XIAO configuration example](https://github.com/yuchamichami/yuPad43/tree/main/firmware/xiao-example)

## Operation

By default, the pointer follows one finger and stops when you lift it. Use two fingers to scroll vertically or horizontally. A quick flick keeps the page moving briefly. Touch the pad again to stop it.

The driver defaults to a coordinate range of 1300 × 1192 with pinch gestures off. The XIAO example sets **scroll level 0 (standard)** in its ZMK input listener.

The electrode configuration is specific to yuPad43. The calibration target is 400, touch and release thresholds are 30 and 26, and Finger Split Factor is 0. Operation has been checked with a 1 mm PLA cover and a XIAO nRF52840 connected over USB.

## Horizontal modes and gesture event contract

Kconfig choice `INPUT_IQS9151_2F_HORIZONTAL_MODE` selects exactly one mode.
The names below have the prefix `CONFIG_INPUT_IQS9151_`.

| Mode | Horizontal output |
| --- | --- |
| `2F_HORIZONTAL_SCROLL` (default) | Horizontal wheel and its inertia when enabled |
| `2F_HORIZONTAL_SWIPE` | Dedicated left/right gestures; no horizontal wheel or inertia |
| `2F_HORIZONTAL_DISABLED` | Neither horizontal wheel nor two-finger horizontal gestures |

Vertical scrolling is independently controlled by `SCROLL_Y_ENABLE`.
The old `SCROLL_X_ENABLE` and prototype `2F_HORIZONTAL_SWIPE_ENABLE` are removed.

In swipe mode, the dominant axis locks at `2F_SCROLL_START_MOVE` (default 50).
A horizontal gesture fires once it also reaches `2F_HORIZONTAL_SWIPE_THRESHOLD`
(default 200 output coordinate counts). All fingers must lift before another
gesture. Staggered lifts cannot move the cursor. Vertical scrolling and its
inertia, pinch priority when enabled, and tap/recontact dragging are preserved.

The driver identifies gestures without assigning actions such as Back/Forward.
The public header is `include/dt-bindings/input/iqs9151.h`.

| Gesture | Event code name |
| --- | --- |
| Two-finger left | `IQS9151_GESTURE_2F_SWIPE_LEFT` |
| Two-finger right | `IQS9151_GESTURE_2F_SWIPE_RIGHT` |
| Three-finger left | `IQS9151_GESTURE_3F_SWIPE_LEFT` |
| Three-finger right | `IQS9151_GESTURE_3F_SWIPE_RIGHT` |
| Three-finger up | `IQS9151_GESTURE_3F_SWIPE_UP` |
| Three-finger down | `IQS9151_GESTURE_3F_SWIPE_DOWN` |

Each emits an `INPUT_EV_KEY` press (value=1) followed by a release (value=0),
both with sync=true. Codes are driver-local identities, not HID usages or mouse
buttons. Directions follow rotated coordinates (left=-X, up=-Y). Unassigned
gestures perform no host action; firmware input processors assign behaviors.
Three-finger swipes now use these events instead of `INPUT_BTN_3` through
`INPUT_BTN_6`. Tap buttons `INPUT_BTN_0` through `INPUT_BTN_2` and the pinch
modifier `INPUT_BTN_7` are unchanged.

## License

Based on ShiniNet / zmk-driver-iqs9151 (MIT License), with changes to the electrode configuration, initialization settings, and ATI communication for yuPad43.

Copyright (c) 2026 ShiniNet. See [LICENSE](LICENSE) for the full license text.
