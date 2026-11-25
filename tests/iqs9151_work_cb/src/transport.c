/* Run the production init, GPIO interrupt, I2C frame and recovery workers. */
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>
#include <drivers/input/iqs9151.h>
#include <string.h>

#include "iqs9151_regs.h"

#define DT_DRV_COMPAT azoteq_iqs9151
#define MEM_BASE IQS9151_ADDR_PRODUCT_NUMBER

struct sensor {
    const struct gpio_dt_spec ready;
    uint8_t mem[0x400];
    atomic_t signalled;
    atomic_t initial_frame;
    atomic_t frame_reads;
    atomic_t acknowledgements;
    atomic_t calibrations;
    atomic_t nack_frames;
    atomic_t nack_mapping;
    atomic_t pin_reset_after_restore;
    atomic_t id_reads;
    atomic_t writes;
    atomic_t lose_release;
};

static atomic_t x_reports;
static atomic_t system_queue_io;
static atomic_t left_down;
static atomic_t left_ups;
static atomic_t move_while_held;
#define RETOUCH_REL_X 77

static void reopen_window(struct k_timer *timer) {
    struct sensor *s = k_timer_user_data_get(timer);
    gpio_emul_input_set(s->ready.port, s->ready.pin, 0);
}
K_TIMER_DEFINE(next_id_window, reopen_window, NULL);

int input_report(const struct device *dev, uint8_t type, uint16_t code,
                 int32_t value, bool sync, k_timeout_t timeout) {
    if (dev == DEVICE_DT_GET(DT_NODELABEL(test_other_pad))) {
        if (type == INPUT_EV_KEY && code == INPUT_BTN_0) {
            atomic_set(&left_down, value);
            if (value == 0) {
                atomic_inc(&left_ups);
            }
        }
        if (type == INPUT_EV_REL && code == INPUT_REL_X &&
            value == RETOUCH_REL_X && atomic_get(&left_down)) {
            atomic_inc(&move_while_held);
        }
    }
    if (dev == DEVICE_DT_GET(DT_NODELABEL(test_pad)) &&
        type == INPUT_EV_REL && code == INPUT_REL_X && value == 12) {
        atomic_inc(&x_reports);
    }
    return 0;
}

static uint8_t *at(struct sensor *s, uint16_t reg) {
    return &s->mem[reg - MEM_BASE];
}

static void signal_window(struct sensor *s) {
    atomic_set(&s->signalled, 1);
    gpio_emul_input_set(s->ready.port, s->ready.pin, 1);
    gpio_emul_input_set(s->ready.port, s->ready.pin, 0);
}

static void set_touch(struct sensor *s, bool down, uint16_t x, uint16_t dx) {
    uint16_t flags = down ? IQS9151_TP_FINGER1_CONFIDENCE | 1 : 0;
    if (dx) {
        flags |= IQS9151_TP_MOVEMENT_DETECTED;
    }
    sys_put_le16(flags, at(s, IQS9151_ADDR_TRACKPAD_FLAGS));
    sys_put_le16(x, at(s, IQS9151_ADDR_FINGER1_X));
    sys_put_le16(200, at(s, IQS9151_ADDR_FINGER1_Y));
    sys_put_le16(dx, at(s, IQS9151_ADDR_RELATIVE_X));
}

static void power_on_sensor(struct sensor *s) {
    memset(s->mem, 0, sizeof(s->mem));
    sys_put_le16(IQS9151_PRODUCT_NUMBER, at(s, IQS9151_ADDR_PRODUCT_NUMBER));
    sys_put_le16(IQS9151_INFO_SHOW_RESET, at(s, IQS9151_ADDR_INFO_FLAGS));
}

static int sensor_transfer(const struct emul *emul, struct i2c_msg *msgs, int count, int addr) {
    struct sensor *s = emul->data;
    if (k_current_get() == &k_sys_work_q.thread) {
        atomic_inc(&system_queue_io);
    }
    if ((count != 1 && count != 2) || msgs[0].len < 2) {
        return -EINVAL;
    }
    uint16_t reg = sys_get_le16(msgs[0].buf);
    size_t len = count == 2 ? msgs[1].len : msgs[0].len - 2;
    if (reg < MEM_BASE || reg - MEM_BASE + len > sizeof(s->mem)) {
        return -EINVAL;
    }
    if (count == 2) {
        if (reg == IQS9151_ADDR_PRODUCT_NUMBER) {
            int previous = atomic_inc(&s->id_reads);
            if (addr == 0x59) {
                sys_put_le16(0xabcd, msgs[1].buf);
                return 0;
            }
            if (addr == 0x58 && previous == 0) {
                gpio_emul_input_set(s->ready.port, s->ready.pin, 1);
                k_timer_user_data_set(&next_id_window, s);
                k_timer_start(&next_id_window, K_MSEC(75), K_NO_WAIT);
                return -EIO;
            }
            if (addr == 0x58 && gpio_pin_get_dt(&s->ready) == 0) {
                return -EIO;
            }
        }
        if (reg == IQS9151_ADDR_RELATIVE_X) {
            atomic_inc(&s->frame_reads);
            if (!atomic_get(&s->signalled)) {
                atomic_set(&s->initial_frame, 1);
            }
            if (atomic_get(&s->nack_frames) > 0) {
                atomic_dec(&s->nack_frames);
                return -EIO; /* RDY stays asserted until the host services it. */
            }
            if (atomic_cas(&s->lose_release, 1, 0)) {
                /* Window expires; a different finger arrives while the old
                 * I2C call is still blocked. The worker will see RDY active. */
                k_msleep(100);
                set_touch(s, true, 720, RETOUCH_REL_X);
                signal_window(s);
                return -EIO;
            }
            if (!(sys_get_le16(at(s, IQS9151_ADDR_INFO_FLAGS)) & IQS9151_INFO_SHOW_RESET)) {
                gpio_emul_input_set(s->ready.port, s->ready.pin, 1);
            }
        }
        memcpy(msgs[1].buf, at(s, reg), len);
        return 0;
    }
    atomic_inc(&s->writes);
    if (reg == IQS9151_ADDR_RX_TX_MAPPING && atomic_cas(&s->nack_mapping, 1, 0)) {
        return -EIO;
    }
    memcpy(at(s, reg), msgs[0].buf + 2, len);
    if (reg == IQS9151_ADDR_SYSTEM_CONTROL && len == 2) {
        uint16_t ctrl = sys_get_le16(at(s, reg));
        if (ctrl & IQS9151_SYS_CTRL_SW_RESET) {
            power_on_sensor(s);
        }
        if (ctrl & IQS9151_SYS_CTRL_ACK_RESET) {
            atomic_inc(&s->acknowledgements);
            sys_put_le16(0, at(s, IQS9151_ADDR_INFO_FLAGS));
        }
        if (ctrl & IQS9151_SYS_CTRL_TP_RE_ATI) {
            atomic_inc(&s->calibrations);
        }
        sys_put_le16(0, at(s, reg)); /* ACK, reset and ATI are self-clearing. */
    } else if (reg == IQS9151_ADDR_CONFIG_SETTINGS && len == 2 &&
               (sys_get_le16(at(s, reg)) & IQS9151_CFG_EVENT_MODE) &&
               atomic_get(&s->pin_reset_after_restore)) {
        power_on_sensor(s);
    }
    return 0;
}

static const struct i2c_emul_api api = {.transfer = sensor_transfer};
static int sensor_init(const struct emul *emul, const struct device *bus) {
    power_on_sensor(emul->data);
    return 0;
}

#define IQS_EMUL(n)                                                                                \
    static struct sensor sensor_##n = { .ready = GPIO_DT_SPEC_INST_GET(n, irq_gpios) };             \
    EMUL_DT_INST_DEFINE(n, sensor_init, &sensor_##n, NULL, &api, NULL);
DT_INST_FOREACH_STATUS_OKAY(IQS_EMUL)

static void wait_for_count_at(atomic_t *counter, int expected, int timeout_ms, int line) {
    int64_t deadline = k_uptime_get() + timeout_ms;
    while (atomic_get(counter) < expected && k_uptime_get() < deadline) {
        k_msleep(1);
    }
    zassert_equal(atomic_get(counter), expected, "line %d: expected %d, got %ld",
                  line, expected, (long)atomic_get(counter));
}
#define wait_for_count(counter, expected, timeout_ms) \
    wait_for_count_at(counter, expected, timeout_ms, __LINE__)

ZTEST(iqs9151_transport, test_id_retry_waits_for_window_and_rejects_wrong_part) {
    struct sensor *good = EMUL_DT_GET(DT_NODELABEL(test_other_pad))->data;
    struct sensor *wrong = EMUL_DT_GET(DT_NODELABEL(test_wrong_part))->data;
    zassert_true(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(test_other_pad))));
    zassert_equal(atomic_get(&good->id_reads), 2);
    zassert_false(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(test_wrong_part))));
    zassert_equal(atomic_get(&wrong->id_reads), 1);
    zassert_equal(atomic_get(&wrong->writes), 0);
}

static void send_touch(struct sensor *s, bool down, uint16_t x, uint16_t dx) {
    int frames = atomic_get(&s->frame_reads);
    set_touch(s, down, x, dx);
    signal_window(s);
    wait_for_count(&s->frame_reads, frames + 1, 100);
}

ZTEST(iqs9151_transport, test_drag_continuity_depends_on_retry_window) {
    struct sensor *s = EMUL_DT_GET(DT_NODELABEL(test_other_pad))->data;
    send_touch(s, true, 300, 0);
    send_touch(s, false, 0, 0);
    wait_for_count(&left_down, 1, 100);
    k_msleep(30);
    send_touch(s, true, 300, 0);
    send_touch(s, true, 400, 5);
    int frames = atomic_get(&s->frame_reads);
    atomic_set(&s->nack_frames, 1);
    signal_window(s);
    wait_for_count(&s->frame_reads, frames + 2, 100);
    k_msleep(300);
    zassert_equal(atomic_get(&left_ups), 0, "same-window retry interrupted a static drag");

    set_touch(s, false, 0, 0);
    atomic_set(&s->lose_release, 1);
    signal_window(s);
    wait_for_count(&left_ups, 1, 350);
    send_touch(s, true, 800, RETOUCH_REL_X);
    zassert_equal(atomic_get(&move_while_held), 0, "new contact inherited the drag");
    send_touch(s, false, 0, 0);
    zassert_equal(atomic_get(&left_down), 0, "ambiguous contact created a tap");
}

ZTEST(iqs9151_transport, test_irq_error_and_already_asserted_pin) {
    zassert_false(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(test_bad_irq))));
    zassert_true(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(test_pad))));
    struct sensor *s = EMUL_DT_GET(DT_NODELABEL(test_pad))->data;
    /* No edge has been generated since boot: the initial level must be serviced. */
    wait_for_count(&s->initial_frame, 1, 300);
}

ZTEST(iqs9151_transport, test_io_retry_and_runtime_reconfiguration) {
    struct sensor *s = EMUL_DT_GET(DT_NODELABEL(test_pad))->data;
    wait_for_count(&s->initial_frame, 1, 300);
    const struct device *pad = DEVICE_DT_GET(DT_NODELABEL(test_pad));
    int ret;
    for (int attempt = 0; attempt < 100; attempt++) {
        ret = iqs9151_scroll_inertia_set_enabled(pad, false);
        if (ret != -EBUSY) {
            break;
        }
        k_msleep(1);
    }
    zassert_ok(ret);
    int frames = atomic_get(&s->frame_reads);
    atomic_set(&s->nack_frames, 1);
    signal_window(s);
    wait_for_count(&s->frame_reads, frames + 2, 100);
    frames += 2;
    atomic_set(&s->nack_frames, 9);
    signal_window(s);
    k_msleep(40);
    zassert_equal(atomic_get(&s->frame_reads), frames + 3);
    atomic_set(&s->nack_frames, 0);
    /* The bus recovers while RDY is stuck low; there is no new GPIO edge. */
    wait_for_count(&s->frame_reads, frames + 4, 2100);
    frames += 4;
    atomic_set(&s->nack_frames, 9);
    signal_window(s);
    wait_for_count(&s->frame_reads, frames + 3, 100);
    gpio_emul_input_set(s->ready.port, s->ready.pin, 1);
    k_msleep(2100);
    zassert_equal(atomic_get(&s->frame_reads), frames + 3,
                  "An inactive window must not trigger a delayed transfer");
    atomic_set(&s->nack_frames, 0);
    signal_window(s);
    wait_for_count(&s->frame_reads, frames + 4, 50);
    frames += 4;
    atomic_set(&s->nack_frames, 9);
    signal_window(s);
    wait_for_count(&s->frame_reads, frames + 3, 100);
    atomic_set(&s->nack_frames, 0);
    /* A fresh edge during backoff must run immediately. */
    signal_window(s);
    wait_for_count(&s->frame_reads, frames + 4, 50);

    const uint16_t config_base = IQS9151_ADDR_ALP_COMPENSATION;
    uint8_t original[0x200];
    memcpy(original, at(s, config_base), sizeof(original));
    int acks = atomic_get(&s->acknowledgements);
    int atis = atomic_get(&s->calibrations);
    power_on_sensor(s);
    atomic_set(&s->nack_mapping, 1); /* Partial reconfiguration, then retry. */
    signal_window(s);
    wait_for_count(&s->acknowledgements, acks + 1, 400);
    /* The first pad now waits 100 ms after ACK. Another instance can still read. */
    struct sensor *other = EMUL_DT_GET(DT_NODELABEL(test_other_pad))->data;
    int other_frames = atomic_get(&other->frame_reads);
    signal_window(other);
    wait_for_count(&other->frame_reads, other_frames + 1, 50);
    wait_for_count(&s->acknowledgements, acks + 2, 900);
    wait_for_count(&s->calibrations, atis + 1, 900);
    k_msleep(20);
    zassert_mem_equal(at(s, config_base), original, sizeof(original));
    bool enabled = true;
    zassert_ok(iqs9151_scroll_inertia_get_enabled(pad, &enabled));
    zassert_false(enabled, "Sensor reinitialization must preserve runtime setting");
    /* ATI quality warning after recovery must permit ordinary cursor motion. */
    sys_put_le16(IQS9151_INFO_ATI_ERROR, at(s, IQS9151_ADDR_INFO_FLAGS));
    sys_put_le16(IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1,
                 at(s, IQS9151_ADDR_TRACKPAD_FLAGS));
    sys_put_le16(12, at(s, IQS9151_ADDR_RELATIVE_X));
    signal_window(s);
    wait_for_count(&x_reports, 1, 100);
    zassert_equal(atomic_get(&system_queue_io), 0);
}

ZTEST(iqs9151_transport, test_repeated_reset_backs_off_and_recovers) {
    struct sensor *s = EMUL_DT_GET(DT_NODELABEL(test_pad))->data;
    int acks = atomic_get(&s->acknowledgements);
    atomic_set(&s->pin_reset_after_restore, 1);
    power_on_sensor(s);
    signal_window(s);
    wait_for_count(&s->acknowledgements, acks + 3, 1500);
    k_msleep(800);
    zassert_equal(atomic_get(&s->acknowledgements), acks + 3,
                  "A repeated reset without a good frame must back off");
    atomic_set(&s->pin_reset_after_restore, 0);
    wait_for_count(&s->acknowledgements, acks + 4, 5500);
    k_msleep(150);
    zassert_true(sys_get_le16(at(s, IQS9151_ADDR_CONFIG_SETTINGS)) & IQS9151_CFG_EVENT_MODE);
}

ZTEST_SUITE(iqs9151_transport, NULL, NULL, NULL, NULL, NULL);
