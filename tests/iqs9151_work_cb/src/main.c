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

static void scroll_pair_at(struct iqs9151_work_cb_fixture *fixture, int64_t ms, uint16_t y) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U;
    const struct iqs9151_test_frame frame = make_frame(2, flags, 0, 0, 0, 100, y, 250, y);
    iqs9151_test_process_frame(fixture->ctx, &frame, ms);
}

ZTEST_F(iqs9151_work_cb, test_scroll_speed_uses_elapsed_time_and_includes_stationary_reports) {
    int32_t vx, vy;
    int64_t last_motion_ms;
    scroll_pair_at(fixture, 0, 100);
    scroll_pair_at(fixture, 20, 160);
    zassert_true(iqs9151_test_scroll_seed(fixture->ctx, 20, &vx, &vy, &last_motion_ms));
    zassert_equal(vy, 30 * 256, "60 counts / 20 ms is 30 counts per tick");
    scroll_pair_at(fixture, 27, 181);
    zassert_true(iqs9151_test_scroll_seed(fixture->ctx, 27, &vx, &vy, &last_motion_ms));
    zassert_equal(vy, 30 * 256, "Same speed across uneven sample intervals");
    scroll_pair_at(fixture, 40, 181);
    zassert_true(iqs9151_test_scroll_seed(fixture->ctx, 40, &vx, &vy, &last_motion_ms));
    zassert_equal(vy, 81 * 10 * 256 / 40, "Stationary intervals reduce the launch speed");
    zassert_equal(last_motion_ms, 27);
    zassert_false(iqs9151_test_scroll_seed(fixture->ctx, 63, &vx, &vy, &last_motion_ms));
}

ZTEST_F(iqs9151_work_cb, test_scroll_glide_uses_elapsed_time_without_a_catchup_burst) {
    const struct iqs9151_test_frame up = {0};
    scroll_pair_at(fixture, 0, 100);
    scroll_pair_at(fixture, 10, 200);
    iqs9151_test_process_frame(fixture->ctx, &up, 11);
    zassert_true(iqs9151_test_scroll_inertia_active(fixture->ctx));

    zassert_equal(iqs9151_test_scroll_wheel_at(fixture->ctx, 9), 0);
    zassert_equal(iqs9151_test_scroll_wheel_at(fixture->ctx, 11), 30);
    zassert_equal(iqs9151_test_scroll_wheel_at(fixture->ctx, 19), 0);
    zassert_equal(iqs9151_test_scroll_wheel_at(fixture->ctx, 20), 28);
    const int32_t late = iqs9151_test_scroll_wheel_at(fixture->ctx, 100);
    zassert_true(late > 0 && late < 2 * 28, "Catch-up burst: %d", late);
    zassert_equal(iqs9151_test_scroll_wheel_at(fixture->ctx, 3100), 0);
    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx));
}

ZTEST_F(iqs9151_work_cb, test_scroll_reversal_uses_only_the_new_direction) {
    int32_t vx, vy;
    int64_t last_motion_ms;
    scroll_pair_at(fixture, 0, 100);
    scroll_pair_at(fixture, 10, 180);
    scroll_pair_at(fixture, 20, 178);
    zassert_false(iqs9151_test_scroll_seed(fixture->ctx, 20, &vx, &vy, &last_motion_ms));
    scroll_pair_at(fixture, 30, 166);
    zassert_true(iqs9151_test_scroll_seed(fixture->ctx, 30, &vx, &vy, &last_motion_ms));
    zassert_equal(vx, 0);
    zassert_equal(vy, -7 * 256);
    zassert_equal(last_motion_ms, 30);
}

ZTEST_F(iqs9151_work_cb, test_scroll_speed_does_not_bridge_a_missing_slot_or_stale_pair) {
    const struct iqs9151_test_frame missing =
        make_frame(2, IQS9151_TP_FINGER1_CONFIDENCE | 2, 0, 0, 0, 100, 170, UINT16_MAX, UINT16_MAX);
    int32_t vx, vy;
    int64_t last_motion_ms;
    scroll_pair_at(fixture, 0, 100);
    scroll_pair_at(fixture, 10, 160);
    iqs9151_test_process_frame(fixture->ctx, &missing, 20);
    scroll_pair_at(fixture, 30, 260);
    zassert_false(iqs9151_test_scroll_seed(fixture->ctx, 30, &vx, &vy, &last_motion_ms));
    scroll_pair_at(fixture, 40, 265);
    zassert_true(iqs9151_test_scroll_seed(fixture->ctx, 40, &vx, &vy, &last_motion_ms));
    zassert_equal(vy, 5 * 256);
    scroll_pair_at(fixture, 100, 425);
    zassert_false(iqs9151_test_scroll_seed(fixture->ctx, 100, &vx, &vy, &last_motion_ms));
}

ZTEST_F(iqs9151_work_cb, test_pinch_with_anchored_finger_wins_over_midpoint_motion) {
    const struct iqs9151_test_frame up = {0};
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U;
    const struct iqs9151_test_frame down = make_frame(2, flags, 0, 0, 0, 400, 300, 400, 600);
    for (int direction = -1; direction <= 1; direction += 2) {
        const struct iqs9151_test_frame move =
            make_frame(2, flags, 0, 0, 0, 400, 300, 400, 600 + direction * 120);
        const int64_t start = direction < 0 ? 0 : 500;
        fixture->log.count = 0;
        iqs9151_test_process_frame(fixture->ctx, &down, start);
        iqs9151_test_process_frame(fixture->ctx, &move, start + 10);
        iqs9151_test_process_frame(fixture->ctx, &up, start + 20);
        zassert_equal(fixture->log.count, 3U);
        zassert_equal(fixture->log.events[0].code, INPUT_BTN_7);
        zassert_equal(fixture->log.events[0].value, 1);
        zassert_equal(fixture->log.events[1].code, INPUT_REL_WHEEL);
        zassert_true(fixture->log.events[1].value * direction > 0);
        zassert_equal(fixture->log.events[2].code, INPUT_BTN_7);
        zassert_equal(fixture->log.events[2].value, 0);
    }
}

ZTEST_F(iqs9151_work_cb, test_parallel_swipe_with_unequal_travel_scrolls) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U;
    const struct iqs9151_test_frame down = make_frame(2, flags, 0, 0, 0, 400, 300, 400, 600);
    const struct iqs9151_test_frame move = make_frame(2, flags, 0, 0, 0, 400, 370, 400, 780);
    iqs9151_test_process_frame(fixture->ctx, &down, 0);
    iqs9151_test_process_frame(fixture->ctx, &move, 10);
    zassert_equal(fixture->log.count, 1U);
    zassert_equal(fixture->log.events[0].code, INPUT_REL_WHEEL);
    zassert_equal(fixture->log.events[0].value, 125);
}

ZTEST_F(iqs9151_work_cb, test_rotation_preserves_pinch_distance_and_radial_motion_zooms) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U;
    const struct iqs9151_test_frame down = make_frame(2, flags, 0, 0, 0, 250, 500, 750, 500);
    /* Each finger remains 250 counts from the midpoint after rotation. */
    const struct iqs9151_test_frame rotated = make_frame(2, flags, 0, 0, 0, 350, 300, 650, 700);
    const struct iqs9151_test_frame spread = make_frame(2, flags, 0, 0, 0, 260, 180, 740, 820);
    const struct iqs9151_test_frame up = {0};

    iqs9151_test_process_frame(fixture->ctx, &down, 0);
    iqs9151_test_process_frame(fixture->ctx, &rotated, 10);
    zassert_equal(fixture->log.count, 0U, "Rotation alone must not trigger pinch");
    iqs9151_test_process_frame(fixture->ctx, &spread, 20);
    iqs9151_test_process_frame(fixture->ctx, &up, 30);
    zassert_equal(fixture->log.count, 3U);
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_7);
    zassert_equal(fixture->log.events[0].value, 1);
    zassert_equal(fixture->log.events[1].code, INPUT_REL_WHEEL);
    zassert_equal(fixture->log.events[1].value,
                  300 * CONFIG_INPUT_IQS9151_2F_PINCH_WHEEL_GAIN_X10 / (12 * 10));
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_7);
    zassert_equal(fixture->log.events[2].value, 0);
}

#ifdef YUPAD43_TEST_CUSTOM_CONFIG
/* Custom gesture configuration tests. */
ZTEST_F(iqs9151_work_cb, test_vertical_scroll_starts_only_on_vertical_travel) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2;
    const struct iqs9151_test_frame down = make_frame(2, flags, 0, 0, 0, 100, 100, 250, 100);
    const struct iqs9151_test_frame horizontal = make_frame(2, flags, 0, 0, 0, 180, 100, 330, 100);
    const struct iqs9151_test_frame nudge = make_frame(2, flags, 0, 0, 0, 180, 105, 330, 105);
    const struct iqs9151_test_frame vertical = make_frame(2, flags, 0, 0, 0, 180, 165, 330, 165);

    iqs9151_test_process_frame(fixture->ctx, &down, 0);
    iqs9151_test_process_frame(fixture->ctx, &horizontal, 10);
    iqs9151_test_process_frame(fixture->ctx, &nudge, 20);
    zassert_equal(fixture->log.count, 0U, "Disabled X must not arm scrolling on a 5-count Y nudge");
    iqs9151_test_process_frame(fixture->ctx, &vertical, 30);
    zassert_equal(fixture->log.count, 1U);
    zassert_equal(fixture->log.events[0].code, INPUT_REL_WHEEL);
    zassert_equal(fixture->log.events[0].value, 60);
}

ZTEST_F(iqs9151_work_cb, test_disabled_scroll_axis_leaves_pinch_available) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2;
    const struct iqs9151_test_frame down = make_frame(2, flags, 0, 0, 0, 100, 100, 250, 100);
    const struct iqs9151_test_frame horizontal = make_frame(2, flags, 0, 0, 0, 180, 100, 330, 100);
    const struct iqs9151_test_frame spread = make_frame(2, flags, 0, 0, 0, 120, 100, 390, 100);
    const struct iqs9151_test_frame up = {0};

    iqs9151_test_process_frame(fixture->ctx, &down, 0);
    iqs9151_test_process_frame(fixture->ctx, &horizontal, 10);
    iqs9151_test_process_frame(fixture->ctx, &spread, 20);
    iqs9151_test_process_frame(fixture->ctx, &up, 30);
    zassert_equal(fixture->log.count, 3U, "Expected pinch press, wheel, release");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_7);
    zassert_equal(fixture->log.events[0].value, 1);
    zassert_equal(fixture->log.events[1].code, INPUT_REL_WHEEL);
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_7);
    zassert_equal(fixture->log.events[2].value, 0);
}

ZTEST_F(iqs9151_work_cb, test_zero_swipe_threshold_suppresses_three_finger_swipes) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | 3;
    const struct iqs9151_test_frame down = make_frame(3, flags, 0, 0, 0, 200, 200, 0, 0);
    const struct iqs9151_test_frame move = make_frame(3, flags, 0, 0, 0, 600, 200, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &down, 0);
    zassert_equal(fixture->log.count, 0U, "Stationary contacts must not emit a swipe");
    iqs9151_test_process_frame(fixture->ctx, &move, 10);
    zassert_equal(fixture->log.count, 0U, "Threshold zero disables the swipe even when moving");
}


ZTEST_F(iqs9151_work_cb, test_taps_disabled_leave_motion_and_scroll_active) {
    const struct iqs9151_test_frame one =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | 1, 0, 0, 0, 100, 100, UINT16_MAX, UINT16_MAX);
    const struct iqs9151_test_frame move =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1,
                   4, -3, 0, 104, 97, UINT16_MAX, UINT16_MAX);
    const uint16_t pair_flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2;
    const struct iqs9151_test_frame two = make_frame(2, pair_flags, 0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame scroll = make_frame(2, pair_flags, 0, 0, 0, 100, 160, 200, 160);
    const struct iqs9151_test_frame up = {0};

    iqs9151_test_process_frame(fixture->ctx, &one, 1);
    iqs9151_test_process_frame(fixture->ctx, &up, 21);
    zassert_equal(fixture->log.count, 0, "TAP=n must not press a button even with PRESSHOLD=y");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0);
    iqs9151_test_process_frame(fixture->ctx, &one, 50);
    iqs9151_test_process_frame(fixture->ctx, &move, 60);
    iqs9151_test_process_frame(fixture->ctx, &up, 80);
    zassert_equal(fixture->log.count, 2);
    zassert_equal(fixture->log.events[0].code, INPUT_REL_X);
    zassert_equal(fixture->log.events[0].value, 4);
    zassert_equal(fixture->log.events[1].code, INPUT_REL_Y);
    zassert_equal(fixture->log.events[1].value, -3);

    iqs9151_test_process_frame(fixture->ctx, &two, 110);
    iqs9151_test_process_frame(fixture->ctx, &up, 140);
    zassert_equal(fixture->log.count, 2, "2F tap must not press BTN1 either");
    iqs9151_test_process_frame(fixture->ctx, &two, 170);
    iqs9151_test_process_frame(fixture->ctx, &scroll, 180);
    zassert_equal(fixture->log.count, 3);
    zassert_equal(fixture->log.events[2].code, INPUT_REL_WHEEL);
    zassert_equal(fixture->log.events[2].value, 60);
}

#else

ZTEST_F(iqs9151_work_cb, test_show_reset_releases_pinch_and_clears_state) {
    const struct iqs9151_test_frame show_reset_frame =
        make_frame(2U, 2U, 0, 0, IQS9151_INFO_SHOW_RESET, 0, 0, 0, 0);

    iqs9151_test_force_pinch_session(fixture->ctx, true);
    iqs9151_test_process_frame(fixture->ctx, &show_reset_frame, 10);

    zassert_equal(fixture->log.count, 1U, "Unexpected event count");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Not a key event");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_7, "Unexpected key code");
    zassert_equal(fixture->log.events[0].value, 0, "Expected pinch release");
    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx),
                  "Scroll inertia should be inactive");
    zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx),
                  "Cursor inertia should be inactive");
    zassert_equal(iqs9151_test_prev_finger_count(fixture->ctx), 0U,
                  "Previous frame should be reset");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared");
}

ZTEST_F(iqs9151_work_cb, test_one_finger_release_starts_cursor_inertia) {
    const struct iqs9151_test_frame one_start =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   24, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame one_move =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   20, 0, 0, 140, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &one_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &one_move, 10);
    iqs9151_test_process_frame(fixture->ctx, &release, 20);

    zassert_true(iqs9151_test_cursor_inertia_active(fixture->ctx),
                 "Cursor inertia should start on release");
    zassert_equal(iqs9151_test_prev_finger_count(fixture->ctx), 0U,
                  "Previous frame should track release");
    zassert_equal(fixture->log.count, 4U,
                  "Only REL events from movement frames are expected");
    for (size_t i = 0; i < fixture->log.count; i++) {
        zassert_equal(fixture->log.events[i].type, IQS9151_TEST_EVENT_REL,
                      "Unexpected non-REL event at idx %u", (unsigned int)i);
    }
}

ZTEST_F(iqs9151_work_cb, test_one_finger_release_after_stale_gap_does_not_start_cursor_inertia) {
    const struct iqs9151_test_frame one_start =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   24, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame one_move =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   20, 0, 0, 140, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &one_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &one_move, 10);
    iqs9151_test_process_frame(fixture->ctx, &release, 80);

    zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx),
                  "Cursor inertia should stay off after a stale release");
    zassert_equal(fixture->log.count, 4U,
                  "Only REL events from active movement frames are expected");
}

ZTEST_F(iqs9151_work_cb, test_one_finger_release_after_slowdown_does_not_start_cursor_inertia) {
    const struct iqs9151_test_frame fast_start =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   14, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame fast_move =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   12, 0, 0, 112, 100, 0, 0);
    const struct iqs9151_test_frame slow_1 =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   1, 0, 0, 113, 100, 0, 0);
    const struct iqs9151_test_frame slow_2 =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   1, 0, 0, 114, 100, 0, 0);
    const struct iqs9151_test_frame slow_3 =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   1, 0, 0, 115, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &fast_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &fast_move, 10);
    iqs9151_test_process_frame(fixture->ctx, &slow_1, 20);
    iqs9151_test_process_frame(fixture->ctx, &slow_2, 30);
    iqs9151_test_process_frame(fixture->ctx, &slow_3, 40);
    iqs9151_test_process_frame(fixture->ctx, &release, 50);

    zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx),
                  "Cursor inertia should stay off after slowing down before release");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_scroll_reports_and_starts_scroll_inertia) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame two_scroll =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 160, 100, 260, 100);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &two_scroll, 10);
    iqs9151_test_process_frame(fixture->ctx, &release, 20);

    zassert_true(iqs9151_test_scroll_inertia_active(fixture->ctx),
                 "Scroll inertia should start on 2F scroll release");
    zassert_equal(iqs9151_test_prev_finger_count(fixture->ctx), 0U,
                  "Previous frame should track release");
    zassert_equal(fixture->log.count, 1U,
                  "Expected a single horizontal wheel event during scroll");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_REL, "Not a REL event");
    zassert_equal(fixture->log.events[0].code, INPUT_REL_HWHEEL, "Unexpected REL code");
    zassert_equal(fixture->log.events[0].value, -60, "Unexpected HWHEEL delta");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_scroll_release_after_stale_gap_does_not_start_inertia) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame two_scroll =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 160, 100, 260, 100);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &two_scroll, 10);
    iqs9151_test_process_frame(fixture->ctx, &release, 80);

    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx),
                  "Scroll inertia should stay off after a stale release");
}

ZTEST_F(iqs9151_work_cb, test_new_one_finger_touch_cancels_scroll_inertia) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame two_scroll =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 160, 100, 260, 100);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame one_start =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   18, 0, 0, 120, 100, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &two_scroll, 10);
    iqs9151_test_process_frame(fixture->ctx, &release, 20);

    zassert_true(iqs9151_test_scroll_inertia_active(fixture->ctx),
                 "Scroll inertia should start on 2F scroll release");

    iqs9151_test_process_frame(fixture->ctx, &one_start, 30);

    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx),
                  "A new 1F touch should cancel scroll inertia");
    zassert_equal(fixture->log.count, 3U,
                  "Expected one scroll REL event plus one 1F cursor frame");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_REL, "Event[1] not REL");
    zassert_equal(fixture->log.events[1].code, INPUT_REL_X, "Event[1] should be REL_X");
    zassert_equal(fixture->log.events[1].value, 18, "Event[1] unexpected REL_X value");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_REL, "Event[2] not REL");
    zassert_equal(fixture->log.events[2].code, INPUT_REL_Y, "Event[2] should be REL_Y");
    zassert_equal(fixture->log.events[2].value, 0, "Event[2] unexpected REL_Y value");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_scroll_tail_two_to_one_to_zero_suppresses_cursor_path) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame two_scroll =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 160, 100, 260, 100);
    const struct iqs9151_test_frame one_tail_move =
        make_frame(1U,
                   IQS9151_TP_FINGER2_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   11, -9, 0, UINT16_MAX, UINT16_MAX, 260, 140);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &two_scroll, 10);
    iqs9151_test_process_frame(fixture->ctx, &one_tail_move, 20);
    iqs9151_test_process_frame(fixture->ctx, &release, 30);

    zassert_true(iqs9151_test_scroll_inertia_active(fixture->ctx),
                 "Scroll inertia should start on 2F scroll release");
    zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx),
                  "Cursor inertia must stay off for a 2F scroll tail");
    zassert_equal(fixture->log.count, 1U,
                  "Only the 2F scroll REL event should be reported");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_REL, "Not a REL event");
    zassert_equal(fixture->log.events[0].code, INPUT_REL_HWHEEL, "Unexpected REL code");
    zassert_equal(fixture->log.events[0].value, -60, "Unexpected HWHEEL delta");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_pinch_reports_btn7_and_wheel) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame two_pinch =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 60, 100, 240, 100);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &two_pinch, 10);
    iqs9151_test_process_frame(fixture->ctx, &release, 20);

    zassert_equal(fixture->log.count, 3U, "Expected BTN7 press, wheel, BTN7 release");

    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_7, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN7 press");

    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_REL, "Event[1] not rel");
    zassert_equal(fixture->log.events[1].code, INPUT_REL_WHEEL, "Event[1] unexpected code");
    const int32_t expected_wheel =
        (80 * CONFIG_INPUT_IQS9151_2F_PINCH_WHEEL_GAIN_X10) / (12 * 10);
    zassert_equal(fixture->log.events[1].value, expected_wheel, "Event[1] unexpected wheel value");

    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_KEY, "Event[2] not key");
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_7, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 0, "Event[2] should be BTN7 release");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_pinch_tail_two_to_one_to_zero_suppresses_cursor_path) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame two_pinch =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 60, 100, 240, 100);
    const struct iqs9151_test_frame one_tail_move =
        make_frame(1U,
                   IQS9151_TP_FINGER2_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   9, 7, 0, UINT16_MAX, UINT16_MAX, 245, 108);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &two_pinch, 10);
    iqs9151_test_process_frame(fixture->ctx, &one_tail_move, 20);
    iqs9151_test_process_frame(fixture->ctx, &release, 30);

    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx),
                  "Scroll inertia should remain off for a pinch tail");
    zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx),
                  "Cursor inertia must stay off for a 2F pinch tail");
    zassert_equal(fixture->log.count, 3U,
                  "Expected pinch press/wheel/release only");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_7, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN7 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_REL, "Event[1] not rel");
    zassert_equal(fixture->log.events[1].code, INPUT_REL_WHEEL, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value,
                  (80 * CONFIG_INPUT_IQS9151_2F_PINCH_WHEEL_GAIN_X10) / (12 * 10),
                  "Event[1] unexpected wheel value");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_KEY, "Event[2] not key");
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_7, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 0, "Event[2] should be BTN7 release");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_pinch_to_three_finger_releases_btn7) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame two_pinch =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 60, 100, 240, 100);
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 100, 100, 200, 100);

    iqs9151_test_process_frame(fixture->ctx, &two_start, 0);
    iqs9151_test_process_frame(fixture->ctx, &two_pinch, 10);
    iqs9151_test_process_frame(fixture->ctx, &three_start, 20);

    zassert_equal(fixture->log.count, 3U,
                  "Expected BTN7 press, wheel, BTN7 release on 2F->3F transition");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_KEY, "Event[2] not key");
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_7, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 0, "Event[2] should be BTN7 release");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_tap_click_emits_btn1) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN1 press on first 2F tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should remain pressed while waiting second 2F touch");

    k_msleep(CONFIG_INPUT_IQS9151_2F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN1 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN1 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_tap_staggered_release_emits_btn1) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN1 press on first 2F tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should remain pressed while waiting second 2F touch");

    k_msleep(CONFIG_INPUT_IQS9151_2F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN1 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN1 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_tap_one_lead_finger_emits_btn1) {
    const struct iqs9151_test_frame one_lead =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &one_lead, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    k_msleep(10);
    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN1 press on one-lead 2F tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should remain pressed while waiting second 2F touch");

    k_msleep(CONFIG_INPUT_IQS9151_2F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN1 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN1 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_tap_moved_one_lead_does_not_click) {
    const struct iqs9151_test_frame one_lead =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame one_lead_move =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 140, 100, 0, 0);
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 140, 100, 240, 100);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 140, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &one_lead, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_lead_move, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    k_msleep(10);
    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 0U, "Moved one-finger lead should not become 2F tap");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_tap_jitter_no_pinch_emits_btn1) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 2230, 1577, 622, 2198);
    const struct iqs9151_test_frame two_jitter =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 2217, 1580, 630, 2198);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 2217, 1580, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_jitter, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U,
                  "Expected deferred BTN1 press for jittered 2F tap without pinch");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should remain pressed while waiting second 2F touch");

    k_msleep(CONFIG_INPUT_IQS9151_2F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN1 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN1 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_tap_releases_latched_hold) {
    const struct iqs9151_test_frame two_start =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_force_hold_button(fixture->ctx, INPUT_BTN_1);

    iqs9151_test_process_frame(fixture->ctx, &two_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U,
                  "Expected BTN1 release by 2F tap when hold is latched");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 0, "Event[0] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared by 2F tap");
}

ZTEST_F(iqs9151_work_cb, test_one_finger_tap_defers_release_until_timeout) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    zassert_equal(fixture->log.count, 1U, "Expected deferred press on first tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_0, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN0 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_0,
                  "BTN0 should remain pressed while waiting second touch");

    k_msleep(CONFIG_INPUT_IQS9151_1F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_0, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN0 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_one_finger_double_tap_emits_release_then_click) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_touch_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 101, 100, 0, 0);
    const struct iqs9151_test_frame second_touch_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_down, k_uptime_get());
    k_msleep(70);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_up, k_uptime_get());

    zassert_equal(fixture->log.count, 4U,
                  "Expected deferred press release + second click on double-tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_0, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN0 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_0, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should release first click");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_KEY, "Event[2] not key");
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_0, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 1, "Event[2] should be second click press");
    zassert_equal(fixture->log.events[3].type, IQS9151_TEST_EVENT_KEY, "Event[3] not key");
    zassert_equal(fixture->log.events[3].code, INPUT_BTN_0, "Event[3] unexpected code");
    zassert_equal(fixture->log.events[3].value, 0, "Event[3] should be second click release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after double-tap");
}

ZTEST_F(iqs9151_work_cb, test_one_finger_second_touch_drag_releases_on_finger_up) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_touch_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame second_touch_move_far =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 140, 100, 0, 0);
    const struct iqs9151_test_frame second_touch_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_touch_move_far, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_touch_up, k_uptime_get());

    zassert_equal(fixture->log.count, 2U,
                  "Expected press on first tap and release on second-touch finger-up");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_0, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN0 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_0, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN0 release");
    zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx),
                  "Cursor inertia must not start after drag release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after drag release");
}

ZTEST_F(iqs9151_work_cb, test_one_finger_long_press_without_tapdrag_arm_does_not_emit_hold) {
    const struct iqs9151_test_frame one_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame one_hold_check =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &one_down, k_uptime_get());
    k_msleep(180);
    iqs9151_test_process_frame(fixture->ctx, &one_hold_check, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 0U,
                  "Long press without TapDrag arm must not emit BTN0 hold");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should stay clear without TapDrag arm");
}

ZTEST_F(iqs9151_work_cb, test_one_finger_tap_releases_latched_hold) {
    const struct iqs9151_test_frame tap_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_force_hold_button(fixture->ctx, INPUT_BTN_0);

    iqs9151_test_process_frame(fixture->ctx, &tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U,
                  "Expected BTN0 release by tap when hold is latched");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_0, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 0, "Event[0] should be BTN0 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared by tap");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_double_tap_emits_release_then_click) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_tap_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 102, 100, 202, 100);
    const struct iqs9151_test_frame second_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_tap_up, k_uptime_get());

    zassert_equal(fixture->log.count, 4U,
                  "Expected deferred press release + second click on 2F double-tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should release first click");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_KEY, "Event[2] not key");
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_1, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 1, "Event[2] should be second click press");
    zassert_equal(fixture->log.events[3].type, IQS9151_TEST_EVENT_KEY, "Event[3] not key");
    zassert_equal(fixture->log.events[3].code, INPUT_BTN_1, "Event[3] unexpected code");
    zassert_equal(fixture->log.events[3].value, 0, "Event[3] should be second click release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after 2F double-tap");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_double_tap_one_lead_second_touch_emits_release_then_click) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_one_lead_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 102, 100, 0, 0);
    const struct iqs9151_test_frame second_tap_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 102, 100, 202, 100);
    const struct iqs9151_test_frame second_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN1 press after first 2F tap");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should remain pressed while waiting for second 2F touch");

    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_one_lead_down, k_uptime_get());
    zassert_equal(fixture->log.count, 1U,
                  "Second-touch one-lead should not cancel pending 2F deferred click");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should stay pressed during second-touch one-lead");

    k_msleep(12);
    iqs9151_test_process_frame(fixture->ctx, &second_tap_down, k_uptime_get());
    k_msleep(10);
    iqs9151_test_process_frame(fixture->ctx, &second_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_tap_up, k_uptime_get());

    zassert_equal(fixture->log.count, 4U,
                  "Expected deferred press release + second click on one-lead 2F double-tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should release first click");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_KEY, "Event[2] not key");
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_1, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 1, "Event[2] should be second click press");
    zassert_equal(fixture->log.events[3].type, IQS9151_TEST_EVENT_KEY, "Event[3] not key");
    zassert_equal(fixture->log.events[3].code, INPUT_BTN_1, "Event[3] unexpected code");
    zassert_equal(fixture->log.events[3].value, 0, "Event[3] should be second click release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after 2F double-tap");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_second_touch_drag_releases_on_finger_up) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_touch_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame second_touch_move_far =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 150, 100, 250, 100);
    const struct iqs9151_test_frame one_drag_continue =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 150, 100, 0, 0);
    const struct iqs9151_test_frame second_touch_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_touch_move_far, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_drag_continue, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should stay pressed while second-touch drag continues");
    iqs9151_test_process_frame(fixture->ctx, &second_touch_up, k_uptime_get());

    zassert_equal(fixture->log.count, 2U,
                  "Expected press on first 2F tap and release on second-touch finger-up");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after second-touch drag release");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_second_touch_drag_one_lead_keeps_hold_until_finger_up) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_one_lead_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 101, 100, 0, 0);
    const struct iqs9151_test_frame second_touch_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 101, 100, 201, 100);
    const struct iqs9151_test_frame second_touch_move_far =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 150, 100, 250, 100);
    const struct iqs9151_test_frame one_drag_continue =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 150, 100, 0, 0);
    const struct iqs9151_test_frame second_touch_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN1 press on first 2F tap");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should remain pressed while waiting second 2F touch");

    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_one_lead_down, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should stay pressed during second-touch one-lead");

    k_msleep(12);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_touch_move_far, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_drag_continue, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should stay pressed while one-lead second-touch drag continues");
    iqs9151_test_process_frame(fixture->ctx, &second_touch_up, k_uptime_get());

    zassert_equal(fixture->log.count, 2U,
                  "Expected press on first 2F tap and release on one-lead second-touch drag end");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_1, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after second-touch drag release");
}

ZTEST_F(iqs9151_work_cb, test_two_finger_second_touch_drag_two_to_one_reports_cursor_motion) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_touch_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 100, 100, 200, 100);
    const struct iqs9151_test_frame second_touch_move_far =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 150, 100, 250, 100);
    const struct iqs9151_test_frame one_drag_move =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   13, -8, 0, 190, 120, 0, 0);
    const struct iqs9151_test_frame second_touch_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_touch_move_far, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_drag_move, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_1,
                  "BTN1 should stay pressed after 2F->1F transition during TapDrag");
    iqs9151_test_process_frame(fixture->ctx, &second_touch_up, k_uptime_get());

    zassert_equal(fixture->log.count, 4U,
                  "Expected BTN1 press, cursor REL_X/Y on 2->1, and BTN1 release");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN1 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_REL, "Event[1] not rel");
    zassert_equal(fixture->log.events[1].code, INPUT_REL_X, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 13, "Event[1] should report cursor delta X");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_REL, "Event[2] not rel");
    zassert_equal(fixture->log.events[2].code, INPUT_REL_Y, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, -8, "Event[2] should report cursor delta Y");
    zassert_equal(fixture->log.events[3].type, IQS9151_TEST_EVENT_KEY, "Event[3] not key");
    zassert_equal(fixture->log.events[3].code, INPUT_BTN_1, "Event[3] unexpected code");
    zassert_equal(fixture->log.events[3].value, 0, "Event[3] should be BTN1 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after second-touch drag release");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_releases_latched_hold) {
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_force_hold_button(fixture->ctx, INPUT_BTN_2);

    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U,
                  "Expected BTN2 release by 3F tap when hold is latched");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 0, "Event[0] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared by 3F tap");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_double_tap_emits_release_then_click) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_tap_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 502, 500, 0, 0);
    const struct iqs9151_test_frame second_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_tap_up, k_uptime_get());

    zassert_equal(fixture->log.count, 4U,
                  "Expected deferred press release + second click on 3F double-tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should release first click");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_KEY, "Event[2] not key");
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_2, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 1, "Event[2] should be second click press");
    zassert_equal(fixture->log.events[3].type, IQS9151_TEST_EVENT_KEY, "Event[3] not key");
    zassert_equal(fixture->log.events[3].code, INPUT_BTN_2, "Event[3] unexpected code");
    zassert_equal(fixture->log.events[3].value, 0, "Event[3] should be second click release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after 3F double-tap");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_second_touch_drag_releases_on_finger_up) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_touch_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame second_touch_move_far =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 560, 500, 0, 0);
    const struct iqs9151_test_frame second_touch_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_touch_move_far, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should stay pressed while second-touch drag continues");
    iqs9151_test_process_frame(fixture->ctx, &second_touch_up, k_uptime_get());

    zassert_equal(fixture->log.count, 2U,
                  "Expected press on first 3F tap and release on second-touch finger-up");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after second-touch drag release");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_second_touch_drag_staged_entry_keeps_hold_until_finger_up) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_one_down =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame second_two_down =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 500, 500, 620, 500);
    const struct iqs9151_test_frame second_three_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 620, 500);
    const struct iqs9151_test_frame second_three_move_far =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 560, 500, 620, 500);
    const struct iqs9151_test_frame second_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN2 press on first 3F tap");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should remain pressed while waiting second 3F touch");

    k_msleep(60);

    iqs9151_test_process_frame(fixture->ctx, &second_one_down, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should stay pressed during staged second-touch 1F entry");
    zassert_equal(fixture->log.count, 1U,
                  "No extra events should be emitted during staged second-touch 1F entry");

    iqs9151_test_process_frame(fixture->ctx, &second_two_down, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should stay pressed during staged second-touch 2F entry");
    zassert_equal(fixture->log.count, 1U,
                  "No extra events should be emitted during staged second-touch 2F entry");

    iqs9151_test_process_frame(fixture->ctx, &second_three_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_three_move_far, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should stay pressed while staged second-touch drag continues");

    iqs9151_test_process_frame(fixture->ctx, &second_up, k_uptime_get());

    zassert_equal(fixture->log.count, 2U,
                  "Expected press on first 3F tap and release on staged second-touch finger-up");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after staged second-touch drag release");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_second_touch_drag_three_to_one_reports_cursor_motion) {
    const struct iqs9151_test_frame first_tap_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame first_tap_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);
    const struct iqs9151_test_frame second_touch_down =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame second_touch_move_far =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 560, 500, 0, 0);
    const struct iqs9151_test_frame one_drag_move =
        make_frame(1U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
                   -9, 15, 0, 610, 520, 0, 0);
    const struct iqs9151_test_frame second_touch_up =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &first_tap_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &first_tap_up, k_uptime_get());
    k_msleep(60);
    iqs9151_test_process_frame(fixture->ctx, &second_touch_down, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &second_touch_move_far, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_drag_move, k_uptime_get());
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should stay pressed after 3F->1F transition during TapDrag");
    iqs9151_test_process_frame(fixture->ctx, &second_touch_up, k_uptime_get());

    zassert_equal(fixture->log.count, 4U,
                  "Expected BTN2 press, cursor REL_X/Y on 3->1, and BTN2 release");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 deferred press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_REL, "Event[1] not rel");
    zassert_equal(fixture->log.events[1].code, INPUT_REL_X, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, -9, "Event[1] should report cursor delta X");
    zassert_equal(fixture->log.events[2].type, IQS9151_TEST_EVENT_REL, "Event[2] not rel");
    zassert_equal(fixture->log.events[2].code, INPUT_REL_Y, "Event[2] unexpected code");
    zassert_equal(fixture->log.events[2].value, 15, "Event[2] should report cursor delta Y");
    zassert_equal(fixture->log.events[3].type, IQS9151_TEST_EVENT_KEY, "Event[3] not key");
    zassert_equal(fixture->log.events[3].code, INPUT_BTN_2, "Event[3] unexpected code");
    zassert_equal(fixture->log.events[3].value, 0, "Event[3] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Hold button should be cleared after second-touch drag release");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_click_emits_btn2) {
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN2 press on first 3F tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should remain pressed while waiting second 3F touch");

    k_msleep(CONFIG_INPUT_IQS9151_3F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN2 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN2 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_staggered_release_emits_btn2) {
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame two_release =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 500, 500, 620, 500);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN2 press on first 3F tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should remain pressed while waiting second 3F touch");

    k_msleep(CONFIG_INPUT_IQS9151_3F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN2 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN2 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_one_lead_finger_emits_btn2) {
    const struct iqs9151_test_frame one_lead =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &one_lead, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN2 press on one-lead 3F tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should remain pressed while waiting second 3F touch");

    k_msleep(CONFIG_INPUT_IQS9151_3F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN2 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN2 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_moved_one_lead_does_not_click) {
    const struct iqs9151_test_frame one_lead =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame one_lead_move =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 540, 500, 0, 0);
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 540, 500, 0, 0);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 540, 500, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &one_lead, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_lead_move, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 0U, "Moved one-finger lead should not become 3F tap");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_two_lead_fingers_emits_btn2) {
    const struct iqs9151_test_frame two_lead =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 500, 500, 620, 500);
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame two_release =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 500, 500, 620, 500);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_lead, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN2 press on two-lead 3F tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should remain pressed while waiting second 3F touch");

    k_msleep(CONFIG_INPUT_IQS9151_3F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN2 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN2 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_moved_two_lead_does_not_click) {
    const struct iqs9151_test_frame two_lead =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 500, 500, 620, 500);
    const struct iqs9151_test_frame two_lead_move =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 540, 500, 660, 500);
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 540, 500, 0, 0);
    const struct iqs9151_test_frame two_release =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 540, 500, 660, 500);
    const struct iqs9151_test_frame one_release =
        make_frame(1U, IQS9151_TP_FINGER1_CONFIDENCE | 1U, 0, 0, 0, 540, 500, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &two_lead, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_lead_move, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &one_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 0U, "Moved two-finger lead should not become 3F tap");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_step_to_two_then_zero_avoids_btn1) {
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 500, 500, 0, 0);
    const struct iqs9151_test_frame two_release =
        make_frame(2U,
                   IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2U,
                   0, 0, 0, 500, 500, 620, 500);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &two_release, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 1U, "Expected deferred BTN2 press on 3->2->0 tap");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2, "Event[0] unexpected code");
    zassert_equal(fixture->log.events[0].value, 1, "Event[0] should be BTN2 press");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), INPUT_BTN_2,
                  "BTN2 should remain pressed while waiting second 3F touch");

    k_msleep(CONFIG_INPUT_IQS9151_3F_TAPDRAG_GAP_MAX_MS + 30);

    zassert_equal(fixture->log.count, 2U, "Expected timeout release after deferred BTN2 press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, INPUT_BTN_2, "Event[1] unexpected code");
    zassert_equal(fixture->log.events[1].value, 0, "Event[1] should be BTN2 release");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U,
                  "Deferred BTN2 press should be released on timeout");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_swipe_right_emits_gesture) {
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1000, 1000, 0, 0);
    const struct iqs9151_test_frame three_swipe =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1400, 1000, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &three_swipe, k_uptime_get());

    zassert_equal(fixture->log.count, 2U, "Expected right swipe press + release");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, IQS9151_GESTURE_3F_SWIPE_RIGHT, "Unexpected gesture");
    zassert_equal(fixture->log.events[0].value, 1, "Expected press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, IQS9151_GESTURE_3F_SWIPE_RIGHT, "Unexpected gesture");
    zassert_equal(fixture->log.events[1].value, 0, "Expected release");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_swipe_continuous_touch_emits_once) {
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1000, 1000, 0, 0);
    const struct iqs9151_test_frame swipe_step_1 =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1400, 1000, 0, 0);
    const struct iqs9151_test_frame swipe_step_2 =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1800, 1000, 0, 0);
    const struct iqs9151_test_frame swipe_step_3 =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 2200, 1000, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &swipe_step_1, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &swipe_step_2, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &swipe_step_3, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 2U,
                  "Expected a single right swipe while 3 fingers stay on pad");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, IQS9151_GESTURE_3F_SWIPE_RIGHT, "Unexpected gesture");
    zassert_equal(fixture->log.events[0].value, 1, "Expected press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, IQS9151_GESTURE_3F_SWIPE_RIGHT, "Unexpected gesture");
    zassert_equal(fixture->log.events[1].value, 0, "Expected release");
}

ZTEST_F(iqs9151_work_cb, test_three_finger_swipe_left_continuous_touch_emits_once) {
    const struct iqs9151_test_frame three_start =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 2200, 1000, 0, 0);
    const struct iqs9151_test_frame swipe_step_1 =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1800, 1000, 0, 0);
    const struct iqs9151_test_frame swipe_step_2 =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1400, 1000, 0, 0);
    const struct iqs9151_test_frame swipe_step_3 =
        make_frame(3U, IQS9151_TP_FINGER1_CONFIDENCE | 3U, 0, 0, 0, 1000, 1000, 0, 0);
    const struct iqs9151_test_frame release =
        make_frame(0U, 0U, 0, 0, 0, 0, 0, 0, 0);

    iqs9151_test_process_frame(fixture->ctx, &three_start, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &swipe_step_1, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &swipe_step_2, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &swipe_step_3, k_uptime_get());
    iqs9151_test_process_frame(fixture->ctx, &release, k_uptime_get());

    zassert_equal(fixture->log.count, 2U,
                  "Expected a single left swipe while 3 fingers stay on pad");
    zassert_equal(fixture->log.events[0].type, IQS9151_TEST_EVENT_KEY, "Event[0] not key");
    zassert_equal(fixture->log.events[0].code, IQS9151_GESTURE_3F_SWIPE_LEFT, "Unexpected gesture");
    zassert_equal(fixture->log.events[0].value, 1, "Expected press");
    zassert_equal(fixture->log.events[1].type, IQS9151_TEST_EVENT_KEY, "Event[1] not key");
    zassert_equal(fixture->log.events[1].code, IQS9151_GESTURE_3F_SWIPE_LEFT, "Unexpected gesture");
    zassert_equal(fixture->log.events[1].value, 0, "Expected release");
}

/* A quality warning must not suppress otherwise valid cursor frames. */
ZTEST_F(iqs9151_work_cb, test_ati_warning_keeps_cursor_and_release_working) {
    for (int i = 0; i < 3; i++) {
        const struct iqs9151_test_frame frame = make_frame(
            1U, IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U,
            120, -70, i < 2 ? IQS9151_INFO_ATI_ERROR : 0,
            200 + i * 120, 200 - i * 70, 0, 0);
        iqs9151_test_process_frame(fixture->ctx, &frame, i * 10);
    }
    zassert_equal(fixture->log.count, 6U, "Warning must not drop cursor events");
    for (int i = 0; i < 3; i++) {
        zassert_equal(fixture->log.events[2*i].code, INPUT_REL_X);
        zassert_equal(fixture->log.events[2*i].value, 120);
        zassert_equal(fixture->log.events[2*i+1].code, INPUT_REL_Y);
        zassert_equal(fixture->log.events[2*i+1].value, -70);
    }
    const struct iqs9151_test_frame release = make_frame(
        0U, 0U, 0, 0, IQS9151_INFO_ATI_ERROR, 0, 0, 0, 0);
    iqs9151_test_process_frame(fixture->ctx, &release, 200);
    zassert_equal(iqs9151_test_prev_finger_count(fixture->ctx), 0U);
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U);
}

/* Reproduce the frame order seen during one physical finger on the XIAO.
 * Only one scan contained a confident pair, for 12 ms. */
ZTEST_F(iqs9151_work_cb, test_isolated_second_contact_keeps_single_tap) {
    const struct iqs9151_test_frame one =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | 1, 0, 0, 0, 100, 100, UINT16_MAX, UINT16_MAX);
    const struct iqs9151_test_frame pair =
        make_frame(2, IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2,
                   0, 0, 0, 100, 100, 190, 120);
    const struct iqs9151_test_frame up = {0};

    iqs9151_test_process_frame(fixture->ctx, &one, 0);
    iqs9151_test_process_frame(fixture->ctx, &one, 45);
    iqs9151_test_process_frame(fixture->ctx, &pair, 58);
    iqs9151_test_process_frame(fixture->ctx, &one, 70);
    iqs9151_test_process_frame(fixture->ctx, &up, 80);
    zassert_equal(fixture->log.count, 1U);
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_0);
    zassert_equal(fixture->log.events[0].value, 1);
}

ZTEST_F(iqs9151_work_cb, test_invalid_second_slot_drops_stale_motion_and_breaks_confirmation) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2;
    const struct iqs9151_test_frame one =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | 1, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame pair = make_frame(2, flags, 0, 0, 0, 100, 100, 220, 100);
    const struct iqs9151_test_frame incomplete =
        make_frame(2, flags | IQS9151_TP_MOVEMENT_DETECTED, 9, -4, 0, 109, 96, UINT16_MAX, 100);
    const struct iqs9151_test_frame up = {0};

    iqs9151_test_process_frame(fixture->ctx, &one, 0);
    iqs9151_test_process_frame(fixture->ctx, &pair, 10);
    iqs9151_test_process_frame(fixture->ctx, &incomplete, 20);
    iqs9151_test_process_frame(fixture->ctx, &pair, 30);
    zassert_equal(fixture->log.count, 0U, "A raw two-contact report has no usable relative XY");
    const struct iqs9151_test_frame motion =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1,
                   3, -2, 0, 112, 94, UINT16_MAX, UINT16_MAX);
    iqs9151_test_process_frame(fixture->ctx, &motion, 40);
    iqs9151_test_process_frame(fixture->ctx, &up, 45);
    zassert_equal(fixture->log.count, 3U);
    zassert_equal(fixture->log.events[0].code, INPUT_REL_X);
    zassert_equal(fixture->log.events[0].value, 3);
    zassert_equal(fixture->log.events[1].code, INPUT_REL_Y);
    zassert_equal(fixture->log.events[1].value, -2);
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_0);
}

ZTEST_F(iqs9151_work_cb, test_confirmed_second_contact_preserves_initial_scroll_delta) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2;
    const struct iqs9151_test_frame one =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | 1, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame pair = make_frame(2, flags, 0, 0, 0, 100, 100, 220, 100);
    const struct iqs9151_test_frame moved = make_frame(2, flags, 0, 0, 0, 100, 180, 220, 180);

    iqs9151_test_process_frame(fixture->ctx, &one, 0);
    iqs9151_test_process_frame(fixture->ctx, &pair, 10);
    iqs9151_test_process_frame(fixture->ctx, &moved, 20);
    zassert_equal(fixture->log.count, 1U);
    zassert_equal(fixture->log.events[0].code, INPUT_REL_WHEEL);
    zassert_equal(fixture->log.events[0].value, 80);
}

ZTEST_F(iqs9151_work_cb, test_buffered_second_contact_keeps_original_down_time) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE | 2;
    const struct iqs9151_test_frame one =
        make_frame(1, IQS9151_TP_FINGER1_CONFIDENCE | 1, 0, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame pair = make_frame(2, flags, 0, 0, 0, 100, 100, 220, 100);
    const struct iqs9151_test_frame up = {0};

    iqs9151_test_process_frame(fixture->ctx, &one, 0);
    iqs9151_test_process_frame(fixture->ctx, &pair, 10);
    iqs9151_test_process_frame(fixture->ctx, &pair, 250);
    iqs9151_test_process_frame(fixture->ctx, &up, 260);
    zassert_equal(fixture->log.count, 0U);
}

/* The IQS keeps contact identities in their absolute-coordinate slots when
 * the other finger lifts. UINT16_MAX represents an unoccupied slot. */
static void send_contact_positions(struct iqs9151_work_cb_fixture *fixture, int64_t ms,
                                   uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    const bool first = x1 != UINT16_MAX;
    const bool second = x2 != UINT16_MAX;
    const uint8_t count = (uint8_t)first + (uint8_t)second;
    uint16_t flags = count;
    if (first) {
        flags |= IQS9151_TP_FINGER1_CONFIDENCE;
    }
    if (second) {
        flags |= IQS9151_TP_FINGER2_CONFIDENCE;
    }
    const struct iqs9151_test_frame frame = make_frame(count, flags, 0, 0, 0, x1, y1, x2, y2);
    iqs9151_test_process_frame(fixture->ctx, &frame, ms);
}

ZTEST_F(iqs9151_work_cb, test_staged_contact_excursion_counts_from_single_finger_down) {
    /* XIAO recorded a right click at device ms 278001; physical finger count
     * for this capture is unknown. F1's Y moved from 1143 to 1118 on pair
     * entry and then to 1069. F2 stayed fixed. Scale the captured positions
     * to the suite's 30-count tolerance (the recorded firmware used 50). */
    send_contact_positions(fixture, 0, 25, 686, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 6, 30, 671, 762, 697);
    send_contact_positions(fixture, 12, 33, 661, 762, 697);
    send_contact_positions(fixture, 38, 40, 641, 762, 697);
    send_contact_positions(fixture, 95, 31, 667, 762, 697);
    send_contact_positions(fixture, 101, UINT16_MAX, UINT16_MAX, 762, 697);
    send_contact_positions(fixture, 107, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U, "The first finger moved beyond the tap tolerance");
}

ZTEST_F(iqs9151_work_cb, test_staged_stable_contacts_allow_right_tap_with_slot2_tail) {
    send_contact_positions(fixture, 0, 25, 686, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 10, 30, 671, 762, 697);
    send_contact_positions(fixture, 20, 32, 669, 763, 698);
    send_contact_positions(fixture, 40, UINT16_MAX, UINT16_MAX, 761, 697);
    send_contact_positions(fixture, 50, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 1U);
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_1);
    zassert_equal(fixture->log.events[0].value, 1);
}

ZTEST_F(iqs9151_work_cb, test_pair_returns_to_origin_after_individual_excursion) {
    /* Total Manhattan separation stays at 80, and the centroid moves by 16.
     * Slot 2 moves 36 counts out and then returns to its starting position. */
    send_contact_positions(fixture, 0, 100, 100, 180, 100);
    send_contact_positions(fixture, 10, 120, 104, 160, 64);
    send_contact_positions(fixture, 20, 100, 100, 180, 100);
    send_contact_positions(fixture, 30, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U, "Returning to the origin must not rearm a tap");
}

ZTEST_F(iqs9151_work_cb, test_staggered_release_checks_surviving_slot1_on_first_tail_frame) {
    send_contact_positions(fixture, 0, 100, 100, 180, 100);
    send_contact_positions(fixture, 10, 140, 100, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 20, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U);
}

ZTEST_F(iqs9151_work_cb, test_staggered_release_checks_slot2_while_release_pending) {
    send_contact_positions(fixture, 0, 100, 100, 180, 100);
    send_contact_positions(fixture, 10, UINT16_MAX, UINT16_MAX, 180, 100);
    send_contact_positions(fixture, 20, UINT16_MAX, UINT16_MAX, 180, 140);
    send_contact_positions(fixture, 30, UINT16_MAX, UINT16_MAX, 180, 100);
    send_contact_positions(fixture, 40, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U);
}


static void scroll_down_before_release(struct iqs9151_work_cb_fixture *fixture) {
    send_contact_positions(fixture, 0, 100, 100, 200, 100);
    send_contact_positions(fixture, 10, 100, 165, 200, 165);
    send_contact_positions(fixture, 20, 100, 200, 200, 200);
}

ZTEST_F(iqs9151_work_cb, test_scroll_fling_waits_for_all_fingers) {
    scroll_down_before_release(fixture);
    send_contact_positions(fixture, 30, 100, 200, UINT16_MAX, UINT16_MAX);
    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx),
                  "Momentum must wait while one finger remains on the pad");
    send_contact_positions(fixture, 55, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_true(iqs9151_test_scroll_inertia_active(fixture->ctx),
                 "An all-up at the stale-gap boundary must preserve the flick");
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U);
}

ZTEST_F(iqs9151_work_cb, test_scroll_fling_expires_from_last_motion) {
    scroll_down_before_release(fixture);
    send_contact_positions(fixture, 45, UINT16_MAX, UINT16_MAX, 200, 200);
    send_contact_positions(fixture, 56, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx),
                  "The last moving sample was 36 ms ago even though the first lift was recent");
}

ZTEST_F(iqs9151_work_cb, test_returning_contact_discards_pending_fling) {
    scroll_down_before_release(fixture);
    send_contact_positions(fixture, 30, 100, 200, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 35, 100, 200, 200, 200);
    /* One pair frame is withheld by the second-contact filter. It still
     * interrupts the lift, even if the next report is already all-up. */
    send_contact_positions(fixture, 40, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx));
    zassert_equal(iqs9151_test_hold_button(fixture->ctx), 0U);
}

ZTEST_F(iqs9151_work_cb, test_scroll_brake_touch_does_not_also_click) {
    const struct iqs9151_test_frame up = {0};
    for (uint8_t fingers = 1U; fingers <= 2U; fingers++) {
        iqs9151_work_cb_before(fixture);
        scroll_down_before_release(fixture);
        iqs9151_test_process_frame(fixture->ctx, &up, 30);
        zassert_true(iqs9151_test_scroll_inertia_active(fixture->ctx));
        size_t before = fixture->log.count;
        uint16_t x2 = (fingers == 2U) ? 200U : UINT16_MAX;
        uint16_t y2 = (fingers == 2U) ? 200U : UINT16_MAX;
        send_contact_positions(fixture, 60, 100, 200, x2, y2);
        zassert_false(iqs9151_test_scroll_inertia_active(fixture->ctx));
        iqs9151_test_process_frame(fixture->ctx, &up, 80);
        zassert_equal(fixture->log.count, before, "A stationary brake touch must not click");
        send_contact_positions(fixture, 260, 100, 200, x2, y2);
        iqs9151_test_process_frame(fixture->ctx, &up, 285);
        zassert_equal(fixture->log.count, before + 1U, "The next independent tap must work");
        zassert_equal(fixture->log.events[before].code, fingers == 1U ? INPUT_BTN_0 : INPUT_BTN_1);
    }
}

static void inject_queue_failures(struct iqs9151_work_cb_fixture *fixture, uint16_t button,
                                  int fail_ups, int fail_downs) {
    input_queue.log = &fixture->log;
    input_queue.button = button;
    input_queue.fail_ups = fail_ups;
    input_queue.fail_downs = fail_downs;
    iqs9151_test_set_event_hook(NULL, NULL);
}

ZTEST_F(iqs9151_work_cb, test_button_up_retries_without_more_sensor_interrupts) {
    inject_queue_failures(fixture, INPUT_BTN_0, 2, 0);
    send_contact_positions(fixture, 0, 100, 100, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 40, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 1U);
    k_msleep(CONFIG_INPUT_IQS9151_1F_TAPDRAG_GAP_MAX_MS + 60);
    zassert_equal(fixture->log.count, 2U);
    zassert_equal(fixture->log.events[0].value, 1);
    zassert_equal(fixture->log.events[1].value, 0);
    zassert_equal(input_queue.fail_ups, 0);
}

ZTEST_F(iqs9151_work_cb, test_three_finger_vertical_gestures_have_distinct_codes) {
    const struct {
        uint16_t y;
        uint16_t code;
    } cases[] = {
        {600, IQS9151_GESTURE_3F_SWIPE_UP},
        {1400, IQS9151_GESTURE_3F_SWIPE_DOWN},
    };
    const struct iqs9151_test_frame down =
        make_frame(3, IQS9151_TP_FINGER1_CONFIDENCE | 3, 0, 0, 0, 1000, 1000, 0, 0);
    const struct iqs9151_test_frame up = {0};
    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        fixture->log.count = 0;
        const struct iqs9151_test_frame move =
            make_frame(3, IQS9151_TP_FINGER1_CONFIDENCE | 3, 0, 0, 0,
                       1000, cases[i].y, 0, 0);
        iqs9151_test_process_frame(fixture->ctx, &down, i * 100);
        iqs9151_test_process_frame(fixture->ctx, &move, i * 100 + 10);
        iqs9151_test_process_frame(fixture->ctx, &up, i * 100 + 20);
        zassert_equal(fixture->log.count, 2U);
        zassert_equal(fixture->log.events[0].code, cases[i].code);
        zassert_equal(fixture->log.events[0].value, 1);
        zassert_equal(fixture->log.events[1].code, cases[i].code);
        zassert_equal(fixture->log.events[1].value, 0);
    }
}

ZTEST_F(iqs9151_work_cb, test_gesture_release_retries_for_highest_release_bit) {
    inject_queue_failures(fixture, IQS9151_GESTURE_3F_SWIPE_DOWN, 2, 0);
    const struct iqs9151_test_frame down =
        make_frame(3, IQS9151_TP_FINGER1_CONFIDENCE | 3, 0, 0, 0, 1000, 1000, 0, 0);
    const struct iqs9151_test_frame move =
        make_frame(3, IQS9151_TP_FINGER1_CONFIDENCE | 3, 0, 0, 0, 1000, 1400, 0, 0);
    const struct iqs9151_test_frame up = {0};
    iqs9151_test_process_frame(fixture->ctx, &down, 0);
    iqs9151_test_process_frame(fixture->ctx, &move, 10);
    iqs9151_test_process_frame(fixture->ctx, &up, 20);
    k_msleep(40);
    zassert_equal(fixture->log.count, 2U);
    zassert_equal(fixture->log.events[0].code, IQS9151_GESTURE_3F_SWIPE_DOWN);
    zassert_equal(fixture->log.events[0].value, 1);
    zassert_equal(fixture->log.events[1].code, IQS9151_GESTURE_3F_SWIPE_DOWN);
    zassert_equal(fixture->log.events[1].value, 0);
    zassert_equal(input_queue.fail_ups, 0);
}

ZTEST_F(iqs9151_work_cb, test_pending_release_precedes_repress_of_same_button) {
    const struct iqs9151_test_frame reset = {.info_flags = IQS9151_INFO_SHOW_RESET};
    inject_queue_failures(fixture, INPUT_BTN_0, 1, 0);
    send_contact_positions(fixture, 0, 100, 100, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 20, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    iqs9151_test_process_frame(fixture->ctx, &reset, 40);
    zassert_equal(fixture->log.count, 1U);
    send_contact_positions(fixture, 60, 100, 100, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 80, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    k_msleep(35);
    zassert_equal(fixture->log.count, 3U);
    zassert_equal(fixture->log.events[0].value, 1);
    zassert_equal(fixture->log.events[1].value, 0);
    zassert_equal(fixture->log.events[2].value, 1);
    k_msleep(CONFIG_INPUT_IQS9151_1F_TAPDRAG_GAP_MAX_MS + 10);
    zassert_equal(fixture->log.count, 4U);
    zassert_equal(fixture->log.events[3].value, 0);
}

ZTEST_F(iqs9151_work_cb, test_pinch_edges_order_wheel_and_retry_modifier_release) {
    inject_queue_failures(fixture, INPUT_BTN_7, 2, 1);
    send_contact_positions(fixture, 0, 100, 100, 200, 100);
    send_contact_positions(fixture, 10, 50, 100, 250, 100);
    zassert_equal(fixture->log.count, 0U, "A pinch wheel needs a confirmed modifier press");
    send_contact_positions(fixture, 20, 40, 100, 260, 100);
    zassert_equal(fixture->log.count, 2U);
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_7);
    zassert_equal(fixture->log.events[0].value, 1);
    zassert_equal(fixture->log.events[1].code, INPUT_REL_WHEEL);
    send_contact_positions(fixture, 30, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 40, 100, 100, 200, 100);
    send_contact_positions(fixture, 50, 100, 170, 200, 170);
    zassert_equal(fixture->log.count, 2U, "Do not zoom while trying to scroll");
    k_msleep(35);
    zassert_equal(fixture->log.count, 3U);
    zassert_equal(fixture->log.events[2].code, INPUT_BTN_7);
    zassert_equal(fixture->log.events[2].value, 0);
    send_contact_positions(fixture, 60, 100, 200, 200, 200);
    zassert_equal(fixture->log.events[3].code, INPUT_REL_WHEEL);
}

ZTEST_F(iqs9151_work_cb, test_any_new_raw_touch_stops_cursor_glide) {
    const uint16_t flags = 1 | IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED;
    const struct iqs9151_test_frame moving = make_frame(1, flags, 24, 0, 0, 100, 100, 0, 0);
    const struct iqs9151_test_frame up = {0};
    for (uint8_t fingers = 1; fingers <= 3; fingers++) {
        iqs9151_test_cancel_pending_work(fixture->ctx);
        iqs9151_test_context_init(fixture->ctx, NULL);
        iqs9151_test_process_frame(fixture->ctx, &moving, 0);
        iqs9151_test_process_frame(fixture->ctx, &moving, 10);
        iqs9151_test_process_frame(fixture->ctx, &up, 20);
        zassert_true(iqs9151_test_cursor_inertia_active(fixture->ctx));
        /* Deliberately missing XY on a two-finger wakeup. */
        const struct iqs9151_test_frame still = make_frame(
            fingers, fingers, 0, 0, 0, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
        iqs9151_test_process_frame(fixture->ctx, &still, 30);
        zassert_false(iqs9151_test_cursor_inertia_active(fixture->ctx), "Finger count %u", fingers);
    }
}

ZTEST_F(iqs9151_work_cb, test_two_finger_staggered_lift_obeys_total_tap_deadline) {
    const int64_t limit = CONFIG_INPUT_IQS9151_2F_TAP_MAX_MS;
    send_contact_positions(fixture, 0, 100, 100, 200, 100);
    send_contact_positions(fixture, limit - 10, 100, 100, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, limit + 1, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U);
}

static void start_three_finger_tap(struct iqs9151_work_cb_fixture *fixture) {
    const uint16_t flags = IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE |
                           IQS9151_TP_FINGER3_CONFIDENCE | 3U;
    const struct iqs9151_test_frame down = make_frame(3, flags, 0, 0, 0, 300, 410, 700, 530);
    iqs9151_test_process_frame(fixture->ctx, &down, 0);
}

ZTEST_F(iqs9151_work_cb, test_three_finger_first_lift_checks_surviving_slot1) {
    start_three_finger_tap(fixture);
    send_contact_positions(fixture, 40, 420, 410, 700, 530);
    send_contact_positions(fixture, 60, 420, 410, UINT16_MAX, UINT16_MAX);
    send_contact_positions(fixture, 90, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U);
}

ZTEST_F(iqs9151_work_cb, test_three_finger_last_slot2_motion_cancels_tap) {
    start_three_finger_tap(fixture);
    send_contact_positions(fixture, 40, 300, 410, 700, 530);
    send_contact_positions(fixture, 60, UINT16_MAX, UINT16_MAX, 700, 530);
    send_contact_positions(fixture, 70, UINT16_MAX, UINT16_MAX, 700, 650);
    send_contact_positions(fixture, 90, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U);
}

ZTEST_F(iqs9151_work_cb, test_three_finger_last_slot3_accumulates_relative_motion) {
    start_three_finger_tap(fixture);
    const uint16_t flags = IQS9151_TP_FINGER3_CONFIDENCE | IQS9151_TP_MOVEMENT_DETECTED | 1U;
    const struct iqs9151_test_frame step =
        make_frame(1, flags, 0, CONFIG_INPUT_IQS9151_3F_TAP_MOVE, 0,
                   UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    iqs9151_test_process_frame(fixture->ctx, &step, 50);
    iqs9151_test_process_frame(fixture->ctx, &step, 65);
    send_contact_positions(fixture, 90, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    for (size_t i = 0; i < fixture->log.count; i++) {
        zassert_not_equal(fixture->log.events[i].type, IQS9151_TEST_EVENT_KEY);
    }
}

ZTEST_F(iqs9151_work_cb, test_three_finger_last_slot3_ignores_stale_relative_motion) {
    start_three_finger_tap(fixture);
    const struct iqs9151_test_frame still =
        make_frame(1, IQS9151_TP_FINGER3_CONFIDENCE | 1U, 120, 0, 0,
                   UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    iqs9151_test_process_frame(fixture->ctx, &still, 50);
    send_contact_positions(fixture, 90, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 1U);
    zassert_equal(fixture->log.events[0].code, INPUT_BTN_2);
    zassert_equal(fixture->log.events[0].value, 1);
}

ZTEST_F(iqs9151_work_cb, test_three_finger_tap_distinguishes_second_finger_motion) {
    const uint16_t flags = 3 | IQS9151_TP_FINGER1_CONFIDENCE | IQS9151_TP_FINGER2_CONFIDENCE;
    struct iqs9151_test_frame triple = make_frame(3, flags, 0, 0, 0, 100, 100, 200, 100);
    iqs9151_test_process_frame(fixture->ctx, &triple, 0);
    triple.finger2_x += CONFIG_INPUT_IQS9151_3F_TAP_MOVE + 1;
    iqs9151_test_process_frame(fixture->ctx, &triple, 50);
    send_contact_positions(fixture, 80, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    zassert_equal(fixture->log.count, 0U);
}

#endif

ZTEST_SUITE(iqs9151_work_cb, NULL, iqs9151_work_cb_setup, iqs9151_work_cb_before, NULL, NULL);
