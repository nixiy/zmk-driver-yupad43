/* SPDX-License-Identifier: MIT */
#ifndef YUPAD43_DRIVERS_INPUT_IQS9151_H_
#define YUPAD43_DRIVERS_INPUT_IQS9151_H_

#include <stdbool.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Set driver-generated scroll inertia for one IQS9151 device.
 * Thread context only. Returns 0 on success, -EINVAL for NULL/wrong device,
 * -ENODEV if not ready, -EWOULDBLOCK in ISR context, or -EBUSY if input is
 * processing or called on the device worker; retry/defer to another thread.
 * Failure changes no state.
 * Disabling stops future inertia reports and clears stored launch velocity.
 * Reports already accepted by input/HID queues cannot be recalled.
 * Setting true when already enabled preserves the current glide.
 * No I2C access or persistent storage. The setting survives sensor recovery
 * and retained-RAM sleep, but resets to Kconfig's initial value on MCU reboot.
 */
int iqs9151_scroll_inertia_set_enabled(const struct device *dev, bool enabled);

/** Get the software enable setting, not whether a glide is active.
 * Returns the last successfully applied driver setting, not a register read.
 * Same errors/context as set; NULL output returns -EINVAL. On error the output
 * is unchanged. Before the first set, returns the Kconfig initial value.
 */
int iqs9151_scroll_inertia_get_enabled(const struct device *dev, bool *enabled);

#ifdef __cplusplus
}
#endif
#endif
