/* SPDX-License-Identifier: MIT */
#ifndef DT_BINDINGS_INPUT_IQS9151_H
#define DT_BINDINGS_INPUT_IQS9151_H

/* Driver-local INPUT_EV_KEY codes, shared by the driver and Devicetree.
 * These are gesture identities, not HID usages or mouse button numbers.
 * A gesture emits value 1 followed by 0, each with sync set.
 * Directions refer to the rotated output coordinates: left = -X, up = -Y.
 */
#define IQS9151_GESTURE_2F_SWIPE_LEFT  0x7e00
#define IQS9151_GESTURE_2F_SWIPE_RIGHT 0x7e01
#define IQS9151_GESTURE_3F_SWIPE_LEFT  0x7e10
#define IQS9151_GESTURE_3F_SWIPE_RIGHT 0x7e11
#define IQS9151_GESTURE_3F_SWIPE_UP    0x7e12
#define IQS9151_GESTURE_3F_SWIPE_DOWN  0x7e13

#endif /* DT_BINDINGS_INPUT_IQS9151_H */
