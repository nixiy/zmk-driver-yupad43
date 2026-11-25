#include <drivers/input/iqs9151.h>
#include <zephyr/input/input.h>
#include <zephyr/ztest.h>
#include "iqs9151_regs.h"
#include "iqs9151_test.h"

static union {
    uint64_t align;
    uint8_t bytes[2048];
} context;
static struct device pad;
static atomic_t wheel_reports;
static atomic_t block_report;
K_SEM_DEFINE(report_entered, 0, 1);
K_SEM_DEFINE(report_continue, 0, 1);
K_THREAD_STACK_DEFINE(frame_stack, 2048);
static struct k_thread frame_thread;
static int isr_set_result;
static int isr_get_result;
K_SEM_DEFINE(isr_done, 0, 1);

static void control_from_isr(struct k_timer *timer) {
    bool enabled;
    isr_set_result = iqs9151_scroll_inertia_set_enabled(&pad, false);
    isr_get_result = iqs9151_scroll_inertia_get_enabled(&pad, &enabled);
    k_sem_give(&isr_done);
}
K_TIMER_DEFINE(control_timer, control_from_isr, NULL);

int input_report(const struct device *dev, uint8_t type, uint16_t code,
                 int32_t value, bool sync, k_timeout_t timeout) {
    if (type == INPUT_EV_REL && (code == INPUT_REL_WHEEL || code == INPUT_REL_HWHEEL)) {
        atomic_inc(&wheel_reports);
        if (atomic_cas(&block_report, 1, 0)) {
            k_sem_give(&report_entered);
            k_sem_take(&report_continue, K_FOREVER);
        }
    }
    return 0;
}

static void before(void *unused) {
    zassert_true(iqs9151_test_context_size() <= sizeof(context.bytes));
    iqs9151_test_context_init(context.bytes, &pad);
    iqs9151_test_bind_device(&pad, context.bytes);
    atomic_clear(&wheel_reports);
}

static void after(void *unused) {
    iqs9151_test_cancel_pending_work(context.bytes);
}

static void pair(int64_t ms, uint16_t y) {
    const struct iqs9151_test_frame f = {
        .finger_count = 2,
        .trackpad_flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2,
        .finger1_x = 100, .finger2_x = 250, .finger1_y = y, .finger2_y = y,
    };
    iqs9151_test_process_frame(context.bytes, &f, ms);
}

static void lift(int64_t ms, uint8_t fingers) {
    const struct iqs9151_test_frame f = {.finger_count = fingers};
    iqs9151_test_process_frame(context.bytes, &f, ms);
}

static void scroll(int64_t base) {
    pair(base, 100);
    pair(base + 10, 160);
    pair(base + 20, 240);
}

static void concurrent_frame(void *a, void *b, void *c) {
    scroll(k_uptime_get());
}

ZTEST(runtime_inertia, test_busy_failure_keeps_setting_and_output) {
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, true));
    atomic_set(&block_report, 1);
    k_tid_t tid = k_thread_create(&frame_thread, frame_stack,
        K_THREAD_STACK_SIZEOF(frame_stack), concurrent_frame, NULL, NULL, NULL,
        K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    zassert_ok(k_sem_take(&report_entered, K_SECONDS(1)));
    bool enabled = false;
    int set_result = iqs9151_scroll_inertia_set_enabled(&pad, false);
    int get_result = iqs9151_scroll_inertia_get_enabled(&pad, &enabled);
    k_sem_give(&report_continue);
    zassert_ok(k_thread_join(tid, K_SECONDS(1)));
    zassert_equal(set_result, -EBUSY);
    zassert_equal(get_result, -EBUSY);
    zassert_false(enabled, "Failed get must not overwrite output");
    zassert_ok(iqs9151_scroll_inertia_get_enabled(&pad, &enabled));
    zassert_true(enabled, "Failed set must not change state");
}

ZTEST(runtime_inertia, test_initial_setting_and_errors) {
    bool enabled = !IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_INERTIA_ENABLE);
    zassert_ok(iqs9151_scroll_inertia_get_enabled(&pad, &enabled));
    zassert_equal(enabled, IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_INERTIA_ENABLE));
    zassert_equal(iqs9151_scroll_inertia_set_enabled(NULL, false), -EINVAL);
    zassert_equal(iqs9151_scroll_inertia_get_enabled(&pad, NULL), -EINVAL);
    struct device wrong = {0};
    zassert_equal(iqs9151_scroll_inertia_get_enabled(&wrong, &enabled), -EINVAL);
    struct device_state not_ready = {0};
    struct device unready = pad;
    unready.state = &not_ready;
    enabled = true;
    zassert_equal(iqs9151_scroll_inertia_get_enabled(&unready, &enabled), -ENODEV);
    zassert_true(enabled);
    zassert_equal(iqs9151_scroll_inertia_set_enabled(&unready, false), -ENODEV);
}

ZTEST(runtime_inertia, test_isr_rejected_without_state_change) {
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, true));
    k_timer_start(&control_timer, K_MSEC(1), K_NO_WAIT);
    zassert_ok(k_sem_take(&isr_done, K_SECONDS(1)));
    zassert_equal(isr_set_result, -EWOULDBLOCK);
    zassert_equal(isr_get_result, -EWOULDBLOCK);
    bool enabled = false;
    zassert_ok(iqs9151_scroll_inertia_get_enabled(&pad, &enabled));
    zassert_true(enabled);
}

ZTEST(runtime_inertia, test_device_settings_are_independent) {
    static union { uint64_t align; uint8_t bytes[2048]; } other_context;
    struct device other;
    iqs9151_test_bind_device(&other, other_context.bytes);
    iqs9151_test_context_init(other_context.bytes, &other);
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, false));
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&other, true));
    bool enabled = true;
    zassert_ok(iqs9151_scroll_inertia_get_enabled(&pad, &enabled));
    zassert_false(enabled);
    zassert_ok(iqs9151_scroll_inertia_get_enabled(&other, &enabled));
    zassert_true(enabled);
    iqs9151_test_cancel_pending_work(other_context.bytes);
}

ZTEST(runtime_inertia, test_disable_running_and_reenable) {
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, true));
    int64_t now = k_uptime_get();
    scroll(now);
    lift(now + 21, 0);
    zassert_true(iqs9151_test_scroll_inertia_active(context.bytes));
    int direct_reports = atomic_get(&wheel_reports);
    k_msleep(25);
    zassert_true(atomic_get(&wheel_reports) > direct_reports,
                 "Enabled inertia must actually emit wheel reports after lift");
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, true));
    zassert_true(iqs9151_test_scroll_inertia_active(context.bytes));
    int ret = -EBUSY;
    for (int i = 0; i < 100 && ret == -EBUSY; i++) {
        ret = iqs9151_scroll_inertia_set_enabled(&pad, false);
        if (ret == -EBUSY) {
            k_msleep(1);
        }
    }
    zassert_ok(ret);
    zassert_false(iqs9151_test_scroll_inertia_active(context.bytes));
    int count = atomic_get(&wheel_reports);
    k_msleep(40);
    zassert_equal(atomic_get(&wheel_reports), count);
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, true));
    k_msleep(40);
    zassert_equal(atomic_get(&wheel_reports), count);
    zassert_false(iqs9151_test_scroll_inertia_active(context.bytes));
}

ZTEST(runtime_inertia, test_disabled_scroll_and_pending_lift_are_cleared) {
    for (int i = 0; i < 4; i++) {
        int64_t now = k_uptime_get();
        zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, false));
        zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, false));
        int count = atomic_get(&wheel_reports);
        scroll(now);
        zassert_true(atomic_get(&wheel_reports) > count, "Direct scroll must remain enabled");
        lift(now + 21, 0);
        zassert_false(iqs9151_test_scroll_inertia_active(context.bytes));
        zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, true));
        lift(now + 22, 0);
        zassert_false(iqs9151_test_scroll_inertia_active(context.bytes));
    }
    int64_t now = k_uptime_get();
    scroll(now);
    lift(now + 21, 1); /* Frozen velocity awaiting the remaining finger. */
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, false));
    zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, true));
    lift(now + 22, 0);
    zassert_false(iqs9151_test_scroll_inertia_active(context.bytes));
}

ZTEST(runtime_inertia, test_sensor_reset_preserves_setting) {
    for (int i = 0; i < 2; i++) {
        bool enabled;
        zassert_ok(iqs9151_scroll_inertia_set_enabled(&pad, i != 0));
        const struct iqs9151_test_frame reset = {.info_flags = IQS9151_INFO_SHOW_RESET};
        iqs9151_test_process_frame(context.bytes, &reset, k_uptime_get());
        zassert_ok(iqs9151_scroll_inertia_get_enabled(&pad, &enabled));
        zassert_equal(enabled, i != 0);
    }
}

ZTEST_SUITE(runtime_inertia, NULL, NULL, before, after, NULL);
