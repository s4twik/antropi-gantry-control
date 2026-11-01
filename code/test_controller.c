/* Regression tests for the submitted controller, not a second implementation.
 * Include the source so small internal checks remain visible without a library. */
#define main cell_program_main
#include "main.c"
#undef main

static int checks = 0;
#define CHECK(expr) do { \
    ++checks; \
    if (!(expr)) { \
        fprintf(stderr, "FAIL at line %d: %s\n", __LINE__, #expr); \
        return 1; \
    } \
} while (0)

static void fresh(void) {
    strcpy(scenario, "normal");
    sim_s = 0; pos_x = 0.9; target_x = 0;
    in_start = false; in_reset = false;
    in_estop_ok = true; in_drive_fault = false; in_drive_ready = true;
    in_position = true; in_pose = true; in_home = false;
    in_limit_min = false; in_limit_max = false;
    in_grant1 = false; in_grant2 = false;
    next_req = 100; first_fault = NO_FAULT; completed_cycles = 0;
    stale_ignored = 0; pose_blocked_ticks = 0; reset_refused_count = 0;
    inj_estop = false; inj_drive = false; inj_fired = false;
    faults_cleared = false; mock_start_sent = false;
    early_reset_sent = false; late_reset_sent = false;
    access1_s = 0; access2_s = 0;
    controller_init();
}

static void at(int s, double x) {
    state = s; pos_x = x; homed = true;
    in_position = true; out_servo_on = true;
    in_grant1 = true; in_grant2 = true;
    out_access1 = needs_access1(s); out_access2 = needs_access2(s);
}

int main(void) {
    /* Every active state must independently enforce these fault inputs. */
    for (int s = HOMING_APPROACH; s <= ROBOT_PLACE; ++s) {
        fresh(); state = s; in_estop_ok = false; vel_x = 1; out_velocity = 1;
        step(); CHECK(state == FAULT && fault == FAULT_ESTOP && safe_fault_outputs());
        fresh(); state = s; in_drive_fault = true;
        step(); CHECK(state == FAULT && fault == FAULT_DRIVE && safe_fault_outputs());
        fresh(); state = s; in_drive_ready = false;
        step(); CHECK(state == FAULT && fault == FAULT_NOT_READY && safe_fault_outputs());
        fresh(); state = s; in_limit_min = true;
        step(); CHECK(state == FAULT && safe_fault_outputs());
        fresh(); state = s; in_limit_max = true;
        step(); CHECK(state == FAULT && safe_fault_outputs());
    }

    /* Start pressed or held during FAULT is consumed, not queued. */
    fresh(); go_fault(FAULT_ESTOP);
    in_start = true; step();
    in_reset = true; step();
    CHECK(state == IDLE && !homed && safe_fault_outputs());
    in_reset = false;
    for (int i = 0; i < 20; ++i) step();
    CHECK(state == IDLE && safe_fault_outputs());
    in_start = false; step(); in_start = true; step();
    CHECK(state == HOMING_APPROACH && out_servo_on && !homed);

    /* A pulse during FAULT does not leak into IDLE either. */
    fresh(); go_fault(FAULT_NO_ACK); in_start = true; step();
    in_start = false; in_reset = true; step(); in_reset = false;
    step(); CHECK(state == IDLE && safe_fault_outputs());

    /* Start+Reset together cannot restart, including while already IDLE. */
    fresh(); state = IDLE; in_start = true; in_reset = true; step();
    in_reset = false; step(); CHECK(state == IDLE);
    fresh(); in_start = true; step(); step(); CHECK(state == IDLE);

    /* Reset has no effect in active states; all Start edges there are discarded. */
    fresh(); state = HOMING_APPROACH; out_servo_on = true; in_pose = false;
    in_start = true; in_reset = true; step();
    CHECK(state == HOMING_APPROACH && vel_x == 0);
    go_fault(FAULT_NO_ACK); in_start = false; in_reset = false; step();
    in_reset = true; step(); in_reset = false; step(); CHECK(state == IDLE);

    /* Every observable persistent recovery inhibitor refuses Reset. */
    for (int which = 0; which < 5; ++which) {
        fresh(); go_fault(FAULT_LIMIT);
        if (which == 0) in_estop_ok = false;
        if (which == 1) in_drive_fault = true;
        if (which == 2) in_drive_ready = false;
        if (which == 3) in_limit_min = true;
        if (which == 4) in_limit_max = true;
        in_reset = true; step();
        CHECK(state == FAULT && reset_refused_count == 1 && safe_fault_outputs());
    }

    /* Fault explicitly stops both command and mock motion and clears messages. */
    fresh(); at(ROBOT_PICK, TRAY_X); ask_robot_or_cnc("PICK", TRAY_X, ROBOT_TIME);
    vel_x = 1; out_velocity = 1; stale_ticks = 4; out_access1 = true;
    in_done_id = waiting_for; in_done_actor = ACTOR_ROBOT;
    go_fault(FAULT_ESTOP);
    CHECK(safe_fault_outputs() && reply_id == 0 && in_done_id == 0);
    CHECK(waiting_actor == ACTOR_NONE && in_done_actor == ACTOR_NONE);

    /* Pose permission covers every homing stage and every station movement. */
    int movement[] = {HOMING_APPROACH,HOMING_BACKOFF,HOMING_REAPPROACH,
                      MOVE_TRAY,MOVE_CNC1,MOVE_CNC2,MOVE_OUTPUT};
    for (size_t i = 0; i < sizeof movement / sizeof movement[0]; ++i) {
        fresh(); at(movement[i], 0.9); target_x = 2.5; in_pose = false;
        double old = pos_x;
        for (int j = 0; j < 500; ++j) step();
        CHECK(state == movement[i] && pos_x == old && vel_x == 0 && out_velocity == 0);
        CHECK(home_s == 0 && move_s == 0);
        in_pose = true; step(); CHECK(pos_x != old);
    }
    for (int s = MOVE_TRAY; s <= MOVE_OUTPUT; ++s) {
        if (s != MOVE_TRAY && s != MOVE_CNC1 && s != MOVE_CNC2 && s != MOVE_OUTPUT) continue;
        fresh(); state = s; homed = false; out_servo_on = true;
        step(); CHECK(state == FAULT && fault == FAULT_NOT_HOMED);
    }

    /* Backoff distance is relative, not based on an unhomed absolute position. */
    fresh(); state = HOMING_APPROACH; pos_x = 1.7; out_servo_on = true; in_home = true;
    step(); CHECK(state == HOMING_BACKOFF && backoff_travel == 0);
    in_home = false;
    for (int i = 0; i < 20 && state == HOMING_BACKOFF; ++i) step();
    CHECK(state == HOMING_REAPPROACH && backoff_travel >= BACK_OFF && pos_x >= 1.8);
    fresh(); state = HOMING_BACKOFF; out_servo_on = true; in_home = true;
    for (int i = 0; i < 30 && state != FAULT; ++i) step();
    CHECK(state == FAULT && fault == FAULT_HOMING);
    fresh(); state = HOMING_APPROACH; out_servo_on = true; home_s = HOME_TIMEOUT;
    step(); CHECK(state == FAULT && fault == FAULT_HOMING);
    fresh(); state = HOMING_REAPPROACH; out_servo_on = true; home_travel = HOME_TRAVEL + 0.01;
    step(); CHECK(state == FAULT && fault == FAULT_HOMING);

    /* IDs and actor identity must both match. Zero is never a valid reply. */
    fresh(); at(ROBOT_PICK, TRAY_X); ask_robot_or_cnc("PICK", TRAY_X, ROBOT_TIME);
    int id = waiting_for;
    in_done_id = id - 1; in_done_actor = ACTOR_ROBOT;
    CHECK(!done_yet(ROBOT_TIMEOUT) && waiting_for == id && stale_ignored == 1);
    in_done_id = id; in_done_actor = ACTOR_CNC1;
    CHECK(!done_yet(ROBOT_TIMEOUT) && waiting_for == id && stale_ignored == 2);
    in_done_id = id; in_done_actor = ACTOR_ROBOT;
    CHECK(done_yet(ROBOT_TIMEOUT) && waiting_for == 0);
    in_done_id = id; in_done_actor = ACTOR_ROBOT;
    CHECK(!done_yet(ROBOT_TIMEOUT));
    fresh(); at(ROBOT_PICK, TRAY_X); ask_robot_or_cnc("PICK", TRAY_X, ROBOT_TIME);
    wait_s = ROBOT_TIMEOUT; CHECK(!done_yet(ROBOT_TIMEOUT) && fault == FAULT_NO_ACK);
    fresh(); at(WAIT_CNC2, CNC2_X); ask_robot_or_cnc("MACHINE CNC2", CNC2_X, CNC_TIME);
    CHECK(waiting_actor == ACTOR_CNC2);
    wait_s = CNC_TIMEOUT; CHECK(!done_yet(CNC_TIMEOUT) && fault == FAULT_NO_ACK);

    /* Access stays asserted throughout load/unload, then releases on retreat DONE. */
    fresh(); at(MOVE_CNC1, CNC1_X); go(CNC1_GRANT_LOAD);
    for (int i = 0; i < 30 && state == CNC1_GRANT_LOAD; ++i) { mock_update(); step(); }
    CHECK(state == ROBOT_LOAD1 && out_access1 && in_grant1);
    mock_update(); step(); CHECK(state == ROBOT_LOAD1 && out_access1 && in_grant1);
    in_done_id = waiting_for; in_done_actor = ACTOR_ROBOT; step();
    CHECK(state == WAIT_CNC1 && !out_access1 && waiting_actor == ACTOR_CNC1);
    int access_states[] = {ROBOT_LOAD1, ROBOT_UNLOAD1, ROBOT_LOAD2, ROBOT_UNLOAD2};
    for (size_t i = 0; i < sizeof access_states / sizeof access_states[0]; ++i) {
        int s = access_states[i]; fresh(); at(s, needs_access1(s) ? CNC1_X : CNC2_X);
        in_grant1 = false; in_grant2 = false; step();
        CHECK(state == FAULT && fault == FAULT_GRANT_LOST && safe_fault_outputs());
    }
    fresh(); at(CNC1_GRANT_LOAD, CNC1_X); grant_s = GRANT_TIMEOUT;
    grant_step(false, ROBOT_LOAD1); CHECK(state == FAULT && fault == FAULT_GRANT);

    /* Wrong position, movement or missing in-position inhibit station actions. */
    for (int which = 0; which < 3; ++which) {
        fresh(); at(ROBOT_PICK, TRAY_X);
        if (which == 0) pos_x += 0.1;
        if (which == 1) vel_x = 0.1;
        if (which == 2) in_position = false;
        ask_robot_or_cnc("PICK", TRAY_X, ROBOT_TIME);
        CHECK(state == FAULT && fault == FAULT_OFF_STATION && waiting_for == 0);
    }
    fresh(); at(MOVE_TRAY, TRAY_X); target_x = TRAY_X; in_position = false;
    step(); CHECK(state == MOVE_TRAY && waiting_for == 0);
    move_s = MOVE_TIMEOUT; step(); CHECK(state == FAULT && fault == FAULT_MOTION);
    fresh(); at(ROBOT_PICK, TRAY_X); ask_robot_or_cnc("PICK", TRAY_X, ROBOT_TIME);
    pos_x += 0.02; step(); CHECK(state == FAULT && fault == FAULT_OFF_STATION);

    /* No uncertainty survives idle E-stop/drive loss or a controller reboot. */
    fresh(); state = IDLE; homed = true; in_estop_ok = false; step();
    CHECK(state == IDLE && !homed && !out_servo_on);
    fresh(); at(WAIT_CNC1, CNC1_X); ask_robot_or_cnc("MACHINE CNC1", CNC1_X, CNC_TIME);
    in_start = true; controller_init();
    CHECK(state == INIT && !homed && safe_fault_outputs());
    step(); step(); CHECK(state == IDLE && !homed && waiting_for == 0);
    in_start = false; step(); in_start = true; step(); CHECK(state == HOMING_APPROACH);

    /* Final state returns to IDLE, and a new cycle needs another Start edge. */
    fresh(); homed = true; go(CYCLE_DONE); step();
    CHECK(state == IDLE && homed && completed_cycles == 1 && vel_x == 0);
    step(); CHECK(state == IDLE);
    in_start = true; step(); CHECK(state == MOVE_TRAY && out_servo_on);

    printf("UNIT RESULT: %d assertions passed\n", checks);
    return 0;
}
