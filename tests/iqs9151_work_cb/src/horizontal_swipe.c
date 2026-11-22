#include <zephyr/input/input.h>
#include <zephyr/ztest.h>
#include <dt-bindings/input/iqs9151.h>

#include "iqs9151_regs.h"
#include "iqs9151_test.h"

#include <string.h>

#define IQS9151_TEST_CTX_BUF_SIZE 1536
#define IQS9151_TEST_MAX_EVENTS 32

struct event_log {
    struct iqs9151_test_event events[IQS9151_TEST_MAX_EVENTS];
    size_t count;
};

struct iqs9151_work_cb_fixture {
    uint8_t ctx[IQS9151_TEST_CTX_BUF_SIZE];
    struct event_log log;
};

/* Fault injection at the real input_report() return boundary. */
static struct {
    struct event_log *log;
    uint16_t button;
    int fail_ups;
    int fail_downs;
} input_queue;

static void record_event(const struct iqs9151_test_event *event, void *user_data);

int input_report(const struct device *dev,
                 uint8_t type, uint16_t code, int32_t value, bool sync,
                 k_timeout_t timeout) {
    if (input_queue.log != NULL) {
        if (type == INPUT_EV_KEY && code == input_queue.button) {
            int *failures = value ? &input_queue.fail_downs : &input_queue.fail_ups;
            if (*failures > 0) {
                (*failures)--;
                return -ENOMSG;
            }
        }
        const struct iqs9151_test_event event = {
            .type = type == INPUT_EV_KEY ? IQS9151_TEST_EVENT_KEY : IQS9151_TEST_EVENT_REL,
            .dev = dev, .code = code, .value = value, .sync = sync, .timeout = timeout,
        };
        record_event(&event, input_queue.log);
    }
    return 0;
}

static void record_event(const struct iqs9151_test_event *event, void *user_data) {
    struct event_log *log = (struct event_log *)user_data;

    if (log->count >= IQS9151_TEST_MAX_EVENTS) {
        return;
    }

    log->events[log->count++] = *event;
}

static struct iqs9151_test_frame make_frame(uint8_t finger_count,
                                            uint16_t trackpad_flags,
                                            int16_t rel_x, int16_t rel_y,
                                            uint16_t info_flags,
                                            uint16_t finger1_x, uint16_t finger1_y,
                                            uint16_t finger2_x, uint16_t finger2_y) {
    return (struct iqs9151_test_frame){
        .rel_x = rel_x,
        .rel_y = rel_y,
        .info_flags = info_flags,
        .trackpad_flags = trackpad_flags,
        .finger_count = finger_count,
        .finger1_x = finger1_x,
        .finger1_y = finger1_y,
        .finger2_x = finger2_x,
        .finger2_y = finger2_y,
    };
}

static void *iqs9151_work_cb_setup(void) {
    static struct iqs9151_work_cb_fixture fixture;
    const size_t ctx_size = iqs9151_test_context_size();

    zassert_true(ctx_size <= sizeof(fixture.ctx),
                 "Context buffer too small: %u > %u",
                 (unsigned int)ctx_size,
                 (unsigned int)sizeof(fixture.ctx));
    memset(&fixture, 0, sizeof(fixture));
    iqs9151_test_context_init(fixture.ctx, NULL);
    iqs9151_test_set_event_hook(record_event, &fixture.log);
    return &fixture;
}

static void iqs9151_work_cb_before(void *fixture_ptr) {
    struct iqs9151_work_cb_fixture *fixture =
        (struct iqs9151_work_cb_fixture *)fixture_ptr;

    memset(&input_queue, 0, sizeof(input_queue));
    memset(&fixture->log, 0, sizeof(fixture->log));
    iqs9151_test_cancel_pending_work(fixture->ctx);
    iqs9151_test_context_init(fixture->ctx, NULL);
    iqs9151_test_set_event_hook(record_event, &fixture->log);
}


static void pair_at(struct iqs9151_work_cb_fixture *fixture, int64_t ms, int x, int y) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U;
    const struct iqs9151_test_frame f = make_frame(2, flags, 0, 0, 0, x, y, x + 150, y);
    iqs9151_test_process_frame(fixture->ctx, &f, ms);
}

static void all_up(struct iqs9151_work_cb_fixture *fixture, int64_t ms) {
    const struct iqs9151_test_frame up = {0};
    iqs9151_test_process_frame(fixture->ctx, &up, ms);
}

static void assert_click(struct event_log *log, uint16_t button) {
    zassert_equal(log->count, 2U, "Expected exactly one click, got %u events", (unsigned)log->count);
    zassert_equal(log->events[0].type, IQS9151_TEST_EVENT_KEY);
    zassert_equal(log->events[0].code, button);
    zassert_equal(log->events[0].value, 1);
    zassert_equal(log->events[1].code, button);
    zassert_equal(log->events[1].value, 0);
}

ZTEST_F(iqs9151_work_cb, test_left_gesture_once_until_all_lift) {
    pair_at(fixture, 0, 600, 300);
    pair_at(fixture, 10, 599, 300); /* Confirm second contact. */
    pair_at(fixture, 20, 401, 310);
    zassert_equal(fixture->log.count, 0U);
    pair_at(fixture, 30, 400, 310);
    pair_at(fixture, 40, 200, 350);
    pair_at(fixture, 50, 900, 600); /* Reversal must not emit a second gesture. */
    const struct iqs9151_test_frame tail =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1,
                   20, 10, 0, 920, 610, UINT16_MAX, UINT16_MAX);
    iqs9151_test_process_frame(fixture->ctx, &tail, 60);
    pair_at(fixture, 70, 500, 300); /* Recontact before full lift. */
    pair_at(fixture, 80, 750, 300);
    all_up(fixture, 90);
    assert_click(&fixture->log, IQS9151_GESTURE_2F_SWIPE_LEFT);
    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx));
    zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx));
}

ZTEST_F(iqs9151_work_cb, test_right_gesture_rearms_after_full_lift) {
    for (int i = 0; i < 2; i++) {
        fixture->log.count = 0;
        pair_at(fixture, i * 100, 300, 300);
        pair_at(fixture, i * 100 + 10, 301, 300);
        pair_at(fixture, i * 100 + 20, 500, 300);
        all_up(fixture, i * 100 + 30);
        assert_click(&fixture->log, IQS9151_GESTURE_2F_SWIPE_RIGHT);
    }
}

ZTEST_F(iqs9151_work_cb, test_short_horizontal_motion_does_not_click_or_scroll) {
    pair_at(fixture, 0, 400, 300);
    pair_at(fixture, 10, 401, 300);
    pair_at(fixture, 20, 480, 310);
    pair_at(fixture, 30, 599, 320);
    all_up(fixture, 40);
    zassert_equal(fixture->log.count, 0U);
    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx));
}

ZTEST_F(iqs9151_work_cb, test_vertical_scroll_locks_axis_and_keeps_inertia) {
    pair_at(fixture, 0, 300, 300);
    pair_at(fixture, 10, 300, 301);
    pair_at(fixture, 20, 310, 360);
    pair_at(fixture, 30, 320, 420);
    pair_at(fixture, 40, 560, 480); /* Horizontal drift after vertical lock. */
    all_up(fixture, 41);
    zassert_true(fixture->log.count > 0U);
    for (size_t i = 0; i < fixture->log.count; i++) {
        zassert_equal(fixture->log.events[i].type, IQS9151_TEST_EVENT_REL);
        zassert_equal(fixture->log.events[i].code, INPUT_REL_WHEEL);
    }
    zassert_true(iqs9151_test_scroll_inertia_active(fixture->ctx));
}

ZTEST_F(iqs9151_work_cb, test_horizontal_lock_does_not_turn_into_vertical_scroll) {
    pair_at(fixture, 0, 400, 300);
    pair_at(fixture, 10, 401, 300);
    pair_at(fixture, 20, 470, 310);
    pair_at(fixture, 30, 480, 650);
    all_up(fixture, 40);
    zassert_equal(fixture->log.count, 0U);
}

ZTEST_F(iqs9151_work_cb, test_two_finger_tap_still_right_clicks) {
    pair_at(fixture, 0, 400, 300);
    pair_at(fixture, 10, 400, 300);
    all_up(fixture, 20);
    zassert_equal(fixture->log.count,
                  IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE) ? 1U : 2U);
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1);
    zassert_equal(fixture->log.events[0].value, 1);
    if (!IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
        zassert_equal(fixture->log.events[1].code, INPUT_BTN_1);
        zassert_equal(fixture->log.events[1].value, 0);
    }
}

ZTEST_F(iqs9151_work_cb, test_pinch_still_wins_over_horizontal_midpoint) {
    pair_at(fixture, 0, 400, 300);
    pair_at(fixture, 10, 400, 300);
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2;
    const struct iqs9151_test_frame spread = make_frame(2, flags, 0, 0, 0, 400, 300, 1000, 300);
    iqs9151_test_process_frame(fixture->ctx, &spread, 20);
    all_up(fixture, 30);
    zassert_equal(fixture->log.count, 3U);
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_7);
    zassert_equal(fixture->log.events[1].code, INPUT_REL_WHEEL);
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_7);
}

ZTEST_F(iqs9151_work_cb, test_full_input_queue_does_not_repeat_successful_press) {
    input_queue.log = &fixture->log;
    input_queue.button = IQS9151_GESTURE_2F_SWIPE_RIGHT;
    input_queue.fail_ups = 1;
    iqs9151_test_set_event_hook(NULL, NULL);
    pair_at(fixture, 0, 300, 300);
    pair_at(fixture, 10, 301, 300);
    pair_at(fixture, 20, 500, 300);
    pair_at(fixture, 30, 700, 300);
    all_up(fixture, 40);
    k_sleep(K_MSEC(25));
    assert_click(&fixture->log, IQS9151_GESTURE_2F_SWIPE_RIGHT);
}

ZTEST_F(iqs9151_work_cb, test_missing_second_slot_does_not_trigger_gesture) {
    pair_at(fixture, 0, 300, 300);
    pair_at(fixture, 10, 301, 300);
    pair_at(fixture, 20, 380, 300);
    const struct iqs9151_test_frame missing =
        make_frame(2, IQS9151_TP_FINGER1_CONFIDENCE | 2, 0, 0, 0,
                   900, 300, UINT16_MAX, UINT16_MAX);
    iqs9151_test_process_frame(fixture->ctx, &missing, 30);
    zassert_equal(fixture->log.count, 0U);
    all_up(fixture, 40);
    zassert_equal(fixture->log.count, 0U);
}

ZTEST_F(iqs9151_work_cb, test_reset_clears_swipe_latch) {
    pair_at(fixture, 0, 300, 300);
    pair_at(fixture, 10, 301, 300);
    pair_at(fixture, 20, 380, 300);
    const struct iqs9151_test_frame reset = {.info_flags = IQS9151_INFO_SHOW_RESET};
    iqs9151_test_process_frame(fixture->ctx, &reset, 30);
    pair_at(fixture, 40, 300, 300);
    pair_at(fixture, 50, 301, 300);
    pair_at(fixture, 60, 500, 300);
    all_up(fixture, 70);
    assert_click(&fixture->log, IQS9151_GESTURE_2F_SWIPE_RIGHT);
}

ZTEST_SUITE(iqs9151_work_cb, NULL, iqs9151_work_cb_setup, iqs9151_work_cb_before, NULL, NULL);
