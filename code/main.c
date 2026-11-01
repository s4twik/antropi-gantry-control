/*
 * =====================================================================
 *  Antropi Robotics - Electronics Engineering Intern Assessment
 *  Task 3: control program for a robotic CNC cell.
 *
 *  The cell: a UR5 robot arm rides on a 3 m gantry and serves 2 lathes.
 *  One rod goes:  tray -> CNC1 -> flip 180 deg -> CNC2 -> output tray.
 *
 *  This program runs on a PC. The "hardware" is simulated in Part 3 at
 *  the bottom of this file (that part pretends to be the real sensors,
 *  the robot and the lathes). This is an ideal, constant-speed PC model,
 *  not board firmware. GPIO, pulse timing, input filtering, communication,
 *  drive feedback and machine safety still need a hardware implementation.
 *
 *  HOW TO BUILD AND RUN
 *    gcc -std=c11 -Wall -Wextra -o cnc_cell main.c -lm
 *    ./cnc_cell            normal run, one full cycle
 *    ./cnc_cell stale      a late "done" from an old request is ignored
 *    ./cnc_cell pose       robot pose missing: the move waits, no fault
 *    ./cnc_cell estop      E-stop pressed: fault, safe reset rules shown
 *    ./cnc_cell drive      drive fault: fault, safe reset rules shown
 *    ./cnc_cell ack        one reply never comes: timeout fault
 *    ./cnc_cell home       home sensor dead: homing fails safely
 *    ./cnc_cell runaway    gantry runs away: end limit fault
 *    ./cnc_cell selfcheck  broken sensor wiring caught at power-on
 * =====================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>

/* ---------------- numbers from the task brief ---------------- */
#define TRAY_X      0.3     /* tray position on the rail, metres      */
#define CNC1_X      1.2     /* first lathe position, metres           */
#define CNC2_X      2.5     /* second lathe position, metres          */
#define LIMIT_MIN  (-0.15)  /* left end switch position               */
#define LIMIT_MAX   3.15    /* right end switch position              */
#define SPEED       1.0     /* normal travel speed, m/s (from brief)  */
#define HOME_SPEED  0.25    /* slow speed while homing, m/s           */
#define CREEP_SPEED 0.05    /* very slow final homing speed, m/s      */
#define BACK_OFF    0.10    /* homing: back off this far, metres      */
#define HOME_TIMEOUT 20.0   /* homing must finish within 20 s         */
#define HOME_TRAVEL  3.5    /* homing must finish within 3.5 m travel */
#define ROBOT_TIMEOUT 6.0   /* wait max 6 s for a robot reply         */
#define CNC_TIMEOUT  12.0   /* machining takes longer: wait 12 s      */
#define GRANT_TIMEOUT 10.0  /* wait max 10 s for the lathe door grant */
#define ROBOT_TIME   1.2    /* fake robot needs 1.2 s per job         */
#define CNC_TIME     3.0    /* fake lathe needs 3 s per part          */
#define GRANT_TIME   0.8    /* fake lathe opens its door in 0.8 s     */
#define MOVE_TIMEOUT 20.0   /* permitted-motion time, including settling */
#define TICK         0.05   /* the controller runs every 0.05 s       */

/* ---------------- the states, same as the state diagram ----------------
 * The program is always in exactly ONE of these states. The state says
 * what the cell is doing right now (going home, waiting for the robot,
 * waiting for a lathe, and so on). */
enum {
    INIT, IDLE,
    HOMING_APPROACH, HOMING_BACKOFF, HOMING_REAPPROACH,
    MOVE_TRAY, ROBOT_PICK,
    MOVE_CNC1, CNC1_GRANT_LOAD, ROBOT_LOAD1, WAIT_CNC1,
    CNC1_GRANT_UNLOAD, ROBOT_UNLOAD1, ROBOT_FLIP,
    MOVE_CNC2, CNC2_GRANT_LOAD, ROBOT_LOAD2, WAIT_CNC2,
    CNC2_GRANT_UNLOAD, ROBOT_UNLOAD2,
    MOVE_OUTPUT, ROBOT_PLACE,
    CYCLE_DONE, FAULT
};

/* Original sequence faults plus explicit feedback/permission interlocks. */
enum {
    NO_FAULT, FAULT_ESTOP, FAULT_DRIVE, FAULT_LIMIT, FAULT_NO_ACK,
    FAULT_HOMING, FAULT_OFF_STATION, FAULT_GRANT, FAULT_SELFCHECK,
    FAULT_GRANT_LOST, FAULT_NOT_HOMED, FAULT_NOT_READY, FAULT_MOTION
};

static const char *state_name(int s) {
    static const char *names[] = {
        "INIT","IDLE","HOMING_APPROACH","HOMING_BACKOFF","HOMING_REAPPROACH",
        "MOVE_TRAY","ROBOT_PICK","MOVE_CNC1","CNC1_GRANT_LOAD","ROBOT_LOAD1",
        "WAIT_CNC1","CNC1_GRANT_UNLOAD","ROBOT_UNLOAD1","ROBOT_FLIP",
        "MOVE_CNC2","CNC2_GRANT_LOAD","ROBOT_LOAD2","WAIT_CNC2",
        "CNC2_GRANT_UNLOAD","ROBOT_UNLOAD2","MOVE_OUTPUT","ROBOT_PLACE",
        "CYCLE_DONE","FAULT"
    };
    return names[s];
}

static const char *fault_name(int f) {
    static const char *names[] = {
        "none", "E-stop chain open", "drive fault", "unexpected end limit",
        "missing acknowledgement (ack timeout)",
        "homing timeout or travel bound",
        "action requested while not stopped at station",
        "CNC access grant timeout", "self-check failed (sensor sanity)",
        "CNC access grant lost", "normal move before homing",
        "drive not ready", "motion/in-position timeout"
    };
    return names[f];
}

/* =====================================================================
 *  PART 1 - the variables. Plain globals, nothing hidden.
 * ===================================================================== */

/* controller memory */
int    state = INIT;        /* what the cell is doing now              */
bool   homed = false;       /* true only after homing has finished     */
double target_x = 0;        /* where the gantry is driving to          */
double wait_s = 0;          /* seconds spent waiting for a reply       */
double grant_s = 0;         /* seconds spent waiting for a door grant  */
double home_s = 0;          /* seconds spent homing                    */
double home_travel = 0;     /* metres travelled while homing           */
double state_s = 0;         /* seconds spent in the current state      */
double fault_s = 0;         /* seconds spent in the FAULT state        */
int    fault = NO_FAULT;    /* why we are in FAULT                     */
bool   ever_faulted = false;/* a fault happened and was reset          */
bool   recovered = false;   /* fault -> reset -> back to IDLE          */

/* requests: every job given to the robot or a lathe gets a number.
 * A "done" reply is only believed if its number matches the job we
 * are waiting for. An old number is a stale reply and is ignored.   */
int         next_req = 100; /* last number used, first job is 101      */
int         waiting_for = 0;/* number of the job we are waiting for    */
const char *waiting_what = ""; /* name of that job, for the log        */

/* Ideal simulated position, NOT an encoder readback from the real drive.
 * A hardware port needs commanded pulse counting and separate drive feedback. */
double pos_x = 0.9;         /* gantry position, metres. At power-on
                               this number is just a guess - that is
                               exactly why homing is needed.           */
double vel_x = 0;           /* gantry speed, m/s                       */

/* inputs: the mock hardware fills these in every tick.
 * On the STM32 these would be GPIO pin readings.                    */
bool in_home;               /* home sensor (at position 0)             */
bool in_limit_min;          /* left end switch                         */
bool in_limit_max;          /* right end switch                        */
bool in_estop_ok;           /* true = E-stop chain is closed (safe)    */
bool in_drive_ready;       /* powered drive reports ready             */
bool in_position;          /* separate mocked drive settling feedback */
bool in_drive_fault;        /* true = servo drive reports a fault      */
bool in_pose;               /* true = robot confirms its travel pose   */
bool in_grant1;             /* true = CNC1 allows the robot in         */
bool in_grant2;             /* true = CNC2 allows the robot in         */
int  in_done_id;            /* a "done" arrived this tick, 0 = none    */
bool in_start;              /* sampled Start button level              */
bool in_reset;              /* sampled Reset button level              */

/* A message is (actor, request ID, command); this is not encoded on one
 * 24 V request/DONE wire. CNC access is a separate maintained permission. */
enum { ACTOR_NONE, ACTOR_ROBOT, ACTOR_CNC1, ACTOR_CNC2 };
int waiting_actor = ACTOR_NONE;
int in_done_actor = ACTOR_NONE;
bool out_servo_on = false;
bool out_access1 = false, out_access2 = false;
double out_velocity = 0;
double backoff_travel = 0, move_s = 0;
bool previous_start = false, previous_reset = false;
int first_fault = NO_FAULT;
int completed_cycles = 0, stale_ignored = 0, pose_blocked_ticks = 0;
int reset_refused_count = 0;

/* Hardware adapter calls, implemented by the mock in Part 3. */
void command_motion(double distance, double velocity);
void stop_motion(void);
void cancel_requests(void);
void submit_job(int actor, int id, const char *what, double duration);

/* the scenario being run, chosen on the command line */
char   scenario[32] = "normal";
double sim_s = 0;           /* clock for the whole simulation          */
bool is_scen(const char *s) { return strcmp(scenario, s) == 0; }

void say(const char *tag, const char *msg) {
    printf("[%6.2fs] %-5s %s\n", sim_s, tag, msg);
}

/* =====================================================================
 *  PART 2 - the controller. This is the part that matters.
 * ===================================================================== */

/* Something went wrong. Stop everything, remember why, and wait in
 * the FAULT state until a human resets the cell.                    */
void go_fault(int why) {
    char text[160];
    if (first_fault == NO_FAULT) first_fault = why;
    fault = why;
    homed = false;
    stop_motion();
    out_servo_on = false;
    out_access1 = false;
    out_access2 = false;
    cancel_requests();
    state = FAULT;
    state_s = 0;
    fault_s = 0;
    snprintf(text, sizeof text, "%s. Motion stopped, new actions inhibited.",
             fault_name(why));
    say("FAULT", text);
}

bool at_station(double station_x) {
    return homed && in_drive_ready && in_position &&
           fabs(vel_x) < 0.000001 && fabs(pos_x - station_x) <= 0.005;
}

/* The access request remains high throughout entry, work and retreat. */
bool needs_access1(int s) { return s == ROBOT_LOAD1 || s == ROBOT_UNLOAD1; }
bool needs_access2(int s) { return s == ROBOT_LOAD2 || s == ROBOT_UNLOAD2; }

void ask_robot_or_cnc(const char *what, double station_x, double job_time) {
    if (!at_station(station_x)) {
        go_fault(FAULT_OFF_STATION);
        return;
    }
    if ((needs_access1(state) && (!out_access1 || !in_grant1)) ||
        (needs_access2(state) && (!out_access2 || !in_grant2))) {
        go_fault(FAULT_GRANT_LOST);
        return;
    }
    if (waiting_for != 0 || next_req == INT_MAX) {
        go_fault(FAULT_NO_ACK);
        return;
    }
    waiting_actor = strcmp(what, "MACHINE CNC1") == 0 ? ACTOR_CNC1 :
                    strcmp(what, "MACHINE CNC2") == 0 ? ACTOR_CNC2 : ACTOR_ROBOT;
    waiting_for = ++next_req;
    waiting_what = what;
    wait_s = 0;
    in_done_id = 0;
    in_done_actor = ACTOR_NONE;
    submit_job(waiting_actor, waiting_for, what, job_time);
    char text[128];
    snprintf(text, sizeof text, "Request #%d: %s", waiting_for, what);
    say("REQ", text);
}

/* Wait for the "done" reply of the job we asked for.
 * Returns true when the correct reply arrived.
 * A reply with the wrong number is old news: print it and keep waiting.
 * No reply for too long: that is a fault.                            */
bool done_yet(double timeout) {
    if (in_done_id != 0) {
        int id = in_done_id;
        int actor = in_done_actor;
        in_done_actor = ACTOR_NONE;
        in_done_id = 0;                     /* we have seen this reply */
        if (waiting_for != 0 && id == waiting_for && actor == waiting_actor) {
            char text[96];
            snprintf(text, sizeof text, "DONE #%d accepted.", id);
            say("OK", text);
            waiting_for = 0;
            waiting_actor = ACTOR_NONE;
            waiting_what = "";
            return true;
        }
        stale_ignored++;
        char text[96];
        snprintf(text, sizeof text, "DONE #%d ignored: stale or wrong actor; waiting for #%d.",
                 id, waiting_for);
        say("WARN", text);
    }
    wait_s = wait_s + TICK;
    if (wait_s > timeout) {
        go_fault(FAULT_NO_ACK);
    }
    return false;
}

/* Change to a new state, and do whatever has to happen on entry:
 * a move gets its target, a robot/CNC state sends its request.      */
void go(int new_state) {
    if (needs_access1(state) && !needs_access1(new_state)) out_access1 = false;
    if (needs_access2(state) && !needs_access2(new_state)) out_access2 = false;
    char text[96];
    snprintf(text, sizeof text, "-> %s", state_name(new_state));
    say("STATE", text);
    state = new_state;
    state_s = 0;
    move_s = 0;

    if (new_state == MOVE_TRAY)   target_x = TRAY_X;
    if (new_state == MOVE_CNC1)   target_x = CNC1_X;
    if (new_state == MOVE_CNC2)   target_x = CNC2_X;
    if (new_state == MOVE_OUTPUT) target_x = TRAY_X;
    if (new_state == HOMING_APPROACH) {
        home_s = 0; home_travel = 0; backoff_travel = 0;
    }
    if (new_state == HOMING_BACKOFF) backoff_travel = 0;
    if (new_state == CNC1_GRANT_LOAD || new_state == CNC1_GRANT_UNLOAD) {
        grant_s = 0; out_access1 = true;
    }
    if (new_state == CNC2_GRANT_LOAD || new_state == CNC2_GRANT_UNLOAD) {
        grant_s = 0; out_access2 = true;
    }
    if (new_state == IDLE || new_state == CYCLE_DONE) {
        stop_motion(); out_servo_on = false;
    }
    if (new_state == CYCLE_DONE) completed_cycles++;
    if (new_state == CYCLE_DONE)
        say("OK", "Cycle complete: finished part is in the output tray.");

    /* robot jobs: name, the station it happens at, how long it takes */
    if (new_state == ROBOT_PICK)    ask_robot_or_cnc("PICK",        TRAY_X, ROBOT_TIME);
    if (new_state == ROBOT_LOAD1)   ask_robot_or_cnc("LOAD CNC1",   CNC1_X, ROBOT_TIME);
    if (new_state == ROBOT_UNLOAD1) ask_robot_or_cnc("UNLOAD CNC1", CNC1_X, ROBOT_TIME);
    if (new_state == ROBOT_FLIP)    ask_robot_or_cnc("FLIP",        CNC1_X, ROBOT_TIME);
    if (new_state == ROBOT_LOAD2)   ask_robot_or_cnc("LOAD CNC2",   CNC2_X, ROBOT_TIME);
    if (new_state == ROBOT_UNLOAD2) ask_robot_or_cnc("UNLOAD CNC2", CNC2_X, ROBOT_TIME);
    if (new_state == ROBOT_PLACE)   ask_robot_or_cnc("PLACE",       TRAY_X, ROBOT_TIME);

    /* machining jobs: the gantry just waits at the lathe */
    if (new_state == WAIT_CNC1) ask_robot_or_cnc("MACHINE CNC1", CNC1_X, CNC_TIME);
    if (new_state == WAIT_CNC2) ask_robot_or_cnc("MACHINE CNC2", CNC2_X, CNC_TIME);
}

/* Drive the gantry towards a target at a fixed speed.
 * Returns true when we have arrived.
 * Special rule: no movement at all while the robot has not confirmed
 * its travel pose. That is a wait, not a fault.                      */
bool motion_permission(void) {
    if (!in_pose) {
        stop_motion();
        if (pose_blocked_ticks == 0)
            say("WARN", "Move blocked: robot has not confirmed travel pose.");
        pose_blocked_ticks++;
        return false;
    }
    return true;
}

bool move_to(double target, double speed) {
    if (!homed) { go_fault(FAULT_NOT_HOMED); return false; }
    if (!motion_permission()) return false;
    move_s += TICK;
    if (move_s > MOVE_TIMEOUT) { go_fault(FAULT_MOTION); return false; }
    double left = target - pos_x;
    if (fabs(left) <= 0.000001) {
        stop_motion();
        return in_position;             /* separate drive feedback, not just target count */
    }
    double distance = fmin(fabs(left), speed * TICK);
    if (left < 0) distance = -distance;
    command_motion(distance, left > 0 ? speed : -speed);
    return false;
}

void home_drive(double direction, double speed) {
    if (!motion_permission()) return;
    double distance = direction * speed * TICK;
    command_motion(distance, direction * speed);
    home_travel += fabs(distance);
    if (state == HOMING_BACKOFF) backoff_travel += fabs(distance);
}

/* Grants are maintained 24 V permissions, not numbered DONE messages. */
bool grant_step(bool granted, int next_state) {
    grant_s += TICK;
    if (granted) { grant_s = 0; go(next_state); return true; }
    if (grant_s > GRANT_TIMEOUT) go_fault(FAULT_GRANT);
    return false;
}

bool safety_ok(void) {
    if (state == INIT || state == FAULT) return true;
    if (state == IDLE || state == CYCLE_DONE) {
        if (!in_estop_ok || in_drive_fault || !in_drive_ready ||
            in_limit_min || in_limit_max) {
            homed = false; stop_motion(); out_servo_on = false;
        }
        return true;
    }
    if (!in_estop_ok) { go_fault(FAULT_ESTOP); return false; }
    if (in_drive_fault) { go_fault(FAULT_DRIVE); return false; }
    if (!in_drive_ready) { go_fault(FAULT_NOT_READY); return false; }
    bool homing = state == HOMING_APPROACH || state == HOMING_BACKOFF ||
                  state == HOMING_REAPPROACH;
    if (in_limit_min || in_limit_max) {
        go_fault(homing ? FAULT_HOMING : FAULT_LIMIT);
        return false;
    }
    if ((needs_access1(state) && (!out_access1 || !in_grant1)) ||
        (needs_access2(state) && (!out_access2 || !in_grant2))) {
        go_fault(FAULT_GRANT_LOST); return false;
    }
    double station = 0;
    bool stationary = true;
    switch (state) {
        case ROBOT_PICK: case ROBOT_PLACE: station = TRAY_X; break;
        case CNC1_GRANT_LOAD: case ROBOT_LOAD1: case WAIT_CNC1:
        case CNC1_GRANT_UNLOAD: case ROBOT_UNLOAD1: case ROBOT_FLIP:
            station = CNC1_X; break;
        case CNC2_GRANT_LOAD: case ROBOT_LOAD2: case WAIT_CNC2:
        case CNC2_GRANT_UNLOAD: case ROBOT_UNLOAD2: station = CNC2_X; break;
        default: stationary = false; break;
    }
    if (stationary && !at_station(station)) {
        go_fault(FAULT_OFF_STATION); return false;
    }
    return true;
}

bool recovery_allowed(void) {
    return in_estop_ok && !in_drive_fault && in_drive_ready &&
           !in_limit_min && !in_limit_max;
}

/* Reboot cancels local work. Real devices also need a session/abort handshake
 * before a new cycle; this mock does not resume interrupted machining. */
void controller_init(void) {
    stop_motion();
    cancel_requests();
    state = INIT; fault = NO_FAULT; homed = false;
    out_servo_on = false; out_access1 = false; out_access2 = false;
    state_s = 0; fault_s = 0; home_s = 0; home_travel = 0;
    wait_s = 0; grant_s = 0; move_s = 0; backoff_travel = 0;
    previous_start = in_start; previous_reset = in_reset;
    recovered = false; ever_faulted = false;
}

/* One run of the controller. The main loop calls this every tick. */
void step(void) {
    /* Edges are consumed in every state, so an old Start cannot be queued. */
    bool start_edge = in_start && !previous_start;
    bool reset_edge = in_reset && !previous_reset;
    previous_start = in_start;
    previous_reset = in_reset;
    state_s = state_s + TICK;
    if (!safety_ok()) return;

    if (state == INIT) {
        /* Power-on self check: BOTH end switches on at the same time
         * is physically impossible, so the wiring must be broken.
         * (E-stop and drive state are NOT checked here - in IDLE they
         * only refuse Start, see below.)                            */
        if (in_limit_min && in_limit_max) { go_fault(FAULT_SELFCHECK); return; }
        homed = false;
        go(IDLE);
        say("INFO", "Cell powered up. Position unknown, homing required.");
    }

    else if (state == IDLE) {
        if (start_edge && !in_reset) {
            if (!recovery_allowed()) {
                say("WARN", "Start refused: E-stop, drive or limits not OK.");
            } else if (!homed) {
                out_servo_on = true;
                say("INFO", "Start accepted. Homing first.");
                go(HOMING_APPROACH);
            } else {
                out_servo_on = true;
                say("INFO", "Start accepted.");
                go(MOVE_TRAY);
            }
        }
    }

    /* ---- homing: find position 0 in three small steps ---- */
    else if (state == HOMING_APPROACH) {
        /* step 1: drive slowly towards the home sensor */
        if (!motion_permission()) return;
        home_s = home_s + TICK;
        if (home_s > HOME_TIMEOUT || home_travel > HOME_TRAVEL) go_fault(FAULT_HOMING);
        else if (in_home) {
            stop_motion();
            say("INFO", "Home sensor seen. Backing off.");
            go(HOMING_BACKOFF);
        }
        else home_drive(-1, HOME_SPEED);
    }
    else if (state == HOMING_BACKOFF) {
        /* step 2: back off until the sensor switches off again */
        if (!motion_permission()) return;
        home_s = home_s + TICK;
        if (home_s > HOME_TIMEOUT || home_travel > HOME_TRAVEL) go_fault(FAULT_HOMING);
        else if (in_home && backoff_travel >= 1.5 * BACK_OFF) go_fault(FAULT_HOMING); /* sensor stuck on */
        else if (!in_home && backoff_travel >= BACK_OFF) {
            stop_motion();
            say("INFO", "Backed off. Slow re-approach.");
            go(HOMING_REAPPROACH);
        }
        else home_drive(+1, HOME_SPEED);
    }
    else if (state == HOMING_REAPPROACH) {
        /* step 3: creep back until the sensor switches on again.
         * This second, slow edge is the accurate zero point.        */
        if (!motion_permission()) return;
        home_s = home_s + TICK;
        if (home_s > HOME_TIMEOUT || home_travel > HOME_TRAVEL) go_fault(FAULT_HOMING);
        else if (in_home) {
            pos_x = 0;
            stop_motion();
            homed = true;
            say("OK", "Home latched. Position = 0.000 m. Normal moves enabled.");
            go(MOVE_TRAY);
        }
        else home_drive(-1, CREEP_SPEED);
    }

    /* ---- the normal cycle, following the 10 steps of the brief ---- */
    else if (state == MOVE_TRAY)   { if (move_to(target_x, SPEED)) go(ROBOT_PICK); }
    else if (state == ROBOT_PICK)  { if (done_yet(ROBOT_TIMEOUT) && state == ROBOT_PICK) go(MOVE_CNC1); }

    else if (state == MOVE_CNC1)        { if (move_to(target_x, SPEED)) go(CNC1_GRANT_LOAD); }
    else if (state == CNC1_GRANT_LOAD)  { grant_step(in_grant1, ROBOT_LOAD1); }
    else if (state == ROBOT_LOAD1)      { if (done_yet(ROBOT_TIMEOUT) && state == ROBOT_LOAD1) go(WAIT_CNC1); }
    else if (state == WAIT_CNC1)        { if (done_yet(CNC_TIMEOUT) && state == WAIT_CNC1) go(CNC1_GRANT_UNLOAD); }
    else if (state == CNC1_GRANT_UNLOAD){ grant_step(in_grant1, ROBOT_UNLOAD1); }
    else if (state == ROBOT_UNLOAD1)    { if (done_yet(ROBOT_TIMEOUT) && state == ROBOT_UNLOAD1) go(ROBOT_FLIP); }
    else if (state == ROBOT_FLIP)       { if (done_yet(ROBOT_TIMEOUT) && state == ROBOT_FLIP) go(MOVE_CNC2); }

    else if (state == MOVE_CNC2)        { if (move_to(target_x, SPEED)) go(CNC2_GRANT_LOAD); }
    else if (state == CNC2_GRANT_LOAD)  { grant_step(in_grant2, ROBOT_LOAD2); }
    else if (state == ROBOT_LOAD2)      { if (done_yet(ROBOT_TIMEOUT) && state == ROBOT_LOAD2) go(WAIT_CNC2); }
    else if (state == WAIT_CNC2)        { if (done_yet(CNC_TIMEOUT) && state == WAIT_CNC2) go(CNC2_GRANT_UNLOAD); }
    else if (state == CNC2_GRANT_UNLOAD){ grant_step(in_grant2, ROBOT_UNLOAD2); }
    else if (state == ROBOT_UNLOAD2)    { if (done_yet(ROBOT_TIMEOUT) && state == ROBOT_UNLOAD2) go(MOVE_OUTPUT); }

    else if (state == MOVE_OUTPUT) { if (move_to(target_x, SPEED)) go(ROBOT_PLACE); }
    else if (state == ROBOT_PLACE) { if (done_yet(ROBOT_TIMEOUT) && state == ROBOT_PLACE) go(CYCLE_DONE); }

    else if (state == CYCLE_DONE) {
        go(IDLE);   /* homed stays true; a fresh Start is still required */
    }

    /* ---- fault: wait for a human. Reset is only accepted when the
     * fault cause is really gone, and reset alone never moves
     * anything: after reset we sit in IDLE and wait for Start. ----- */
    else if (state == FAULT) {
        fault_s = fault_s + TICK;
        if (reset_edge) {
            if (!in_estop_ok) {
                reset_refused_count++;
                say("WARN", "Reset refused: E-stop still pressed.");
            } else if (in_drive_fault) {
                reset_refused_count++;
                say("WARN", "Reset refused: drive fault still active.");
            } else if (!recovery_allowed()) {
                reset_refused_count++;
                say("WARN", "Reset refused: drive not ready or an end limit is active.");
            } else {
                cancel_requests();
                stop_motion();
                fault = NO_FAULT;
                ever_faulted = true;
                homed = false;
                say("OK", "Reset done. Nothing moved. Press Start; gantry will re-home first.");
                go(IDLE);
            }
        }
    }
}

/* =====================================================================
 *  PART 3 - the mock hardware. No physical stopping performance or
 *  communication transport is demonstrated by this ideal PC model.
 * ===================================================================== */
int reply_ticks = -1, reply_id = 0, reply_actor = ACTOR_NONE;
int stale_ticks = -1;
bool reply_dropped = false;
bool inj_estop = false, inj_drive = false, inj_fired = false;
bool faults_cleared = false;
bool mock_start_sent = false, early_reset_sent = false, late_reset_sent = false;
double access1_s = 0, access2_s = 0;

void command_motion(double distance, double velocity) {
    if (!out_servo_on) { stop_motion(); return; }
    out_velocity = velocity;
    vel_x = velocity;
    in_position = false;
    /* In this fault scenario only the plant ignores the motion command. */
    if (!(is_scen("runaway") && state == MOVE_CNC1)) pos_x += distance;
}

void stop_motion(void) {
    out_velocity = 0;
    vel_x = 0;                    /* ideal mock stop, not a real stopping model */
}

void cancel_requests(void) {
    waiting_for = 0; waiting_actor = ACTOR_NONE; waiting_what = "";
    in_done_id = 0; in_done_actor = ACTOR_NONE;
    reply_ticks = -1; stale_ticks = -1; reply_id = 0;
    reply_actor = ACTOR_NONE; reply_dropped = false;
}

void submit_job(int actor, int id, const char *what, double duration) {
    reply_actor = actor; reply_id = id;
    reply_ticks = (int)lround(duration / TICK);
    reply_dropped = is_scen("ack") && strcmp(what, "PICK") == 0;
    stale_ticks = -1;
    if ((is_scen("stale") || is_scen("wrong_actor")) && strcmp(what, "PICK") == 0)
        stale_ticks = reply_ticks * 2 / 5;
}

bool home_sensor_now(void) {
    if (!faults_cleared && is_scen("home")) return false;
    if (!faults_cleared && is_scen("stuck")) return true;
    return pos_x <= 0;
}

void mock_update(void) {
    sim_s += TICK;
    in_start = false; in_reset = false;
    in_done_id = 0; in_done_actor = ACTOR_NONE;

    if (is_scen("estop") && state == MOVE_CNC1 && !inj_fired) {
        inj_fired = true; inj_estop = true; say("INJ", "E-stop pressed.");
    }
    if (is_scen("drive") && state == MOVE_TRAY && !inj_fired) {
        inj_fired = true; inj_drive = true; say("INJ", "Drive fault injected.");
    }
    if (is_scen("start_fault") && state == MOVE_TRAY && !inj_fired) {
        inj_fired = true; inj_estop = true; say("INJ", "E-stop pressed; Start held through Reset.");
    }
    /* External motion is applied before sampling the end switches. */
    if (is_scen("runaway") && state == MOVE_CNC1 && !faults_cleared) {
        pos_x += SPEED * TICK; vel_x = SPEED;
    }

    /* Deliberate service is separate from Reset. Repositioning is a manual
     * mock repair, never automatic recovery motion commanded by firmware. */
    if (state == FAULT) {
        if (fault_s >= 1.0 && !early_reset_sent) {
            in_reset = true; early_reset_sent = true;
        }
        if (fault_s >= 2.0 && !faults_cleared) {
            faults_cleared = true; inj_estop = false; inj_drive = false;
            if (pos_x <= LIMIT_MIN || pos_x >= LIMIT_MAX) pos_x = 0.9;
            say("INJ", "Operator clears fault; any manual repositioning is mocked.");
        }
        if (fault_s >= 2.5 && !late_reset_sent) {
            in_reset = true; late_reset_sent = true;
        }
    }
    in_home = home_sensor_now();
    in_limit_min = (!faults_cleared && is_scen("selfcheck")) || pos_x <= LIMIT_MIN;
    in_limit_max = (!faults_cleared && is_scen("selfcheck")) || pos_x >= LIMIT_MAX;
    in_estop_ok = !inj_estop;
    in_drive_fault = inj_drive;
    in_drive_ready = !inj_drive && !(is_scen("notready") && state == MOVE_TRAY && !faults_cleared);
    in_position = fabs(vel_x) < 0.000001 && !(is_scen("inposition") && state == MOVE_TRAY);
    in_pose = !(is_scen("pose") && state == MOVE_TRAY && state_s < 4.0) &&
              !(is_scen("home_pose") && state == HOMING_APPROACH && state_s < 4.0);

    access1_s = out_access1 ? access1_s + TICK : 0;
    access2_s = out_access2 ? access2_s + TICK : 0;
    in_grant1 = out_access1 && access1_s >= GRANT_TIME;
    in_grant2 = out_access2 && access2_s >= GRANT_TIME;
    if (is_scen("grant") && !faults_cleared) in_grant1 = false;
    if (is_scen("grant_loss") && state == ROBOT_LOAD1 && state_s >= 0.3)
        in_grant1 = false;

    if (stale_ticks > 0 && --stale_ticks == 0) {
        in_done_id = is_scen("wrong_actor") ? reply_id : reply_id - 1;
        in_done_actor = is_scen("wrong_actor") ? ACTOR_CNC1 : reply_actor;
    }
    if (reply_ticks > 0 && --reply_ticks == 0 && !reply_dropped) {
        in_done_id = reply_id; in_done_actor = reply_actor;
    }
    if (sim_s >= 1.0 && !mock_start_sent) {
        mock_start_sent = true; in_start = true;
    }
    if (is_scen("start_fault") && first_fault != NO_FAULT) in_start = true;
}

bool safe_fault_outputs(void) {
    return !homed && !out_servo_on && fabs(vel_x) < 0.000001 &&
           fabs(out_velocity) < 0.000001 && waiting_for == 0 &&
           reply_ticks == -1 && stale_ticks == -1 &&
           !out_access1 && !out_access2;
}

int expected_fault(void) {
    if (is_scen("estop") || is_scen("start_fault")) return FAULT_ESTOP;
    if (is_scen("drive")) return FAULT_DRIVE;
    if (is_scen("ack")) return FAULT_NO_ACK;
    if (is_scen("home") || is_scen("stuck")) return FAULT_HOMING;
    if (is_scen("runaway")) return FAULT_LIMIT;
    if (is_scen("selfcheck")) return FAULT_SELFCHECK;
    if (is_scen("grant")) return FAULT_GRANT;
    if (is_scen("grant_loss")) return FAULT_GRANT_LOST;
    if (is_scen("notready")) return FAULT_NOT_READY;
    if (is_scen("inposition")) return FAULT_MOTION;
    return NO_FAULT;
}

int main(int argc, char **argv) {
    const char *valid[] = {"normal", "stale", "pose", "estop", "drive", "ack",
        "home", "runaway", "selfcheck", "home_pose", "grant", "grant_loss",
        "stuck", "notready", "inposition", "start_fault", "wrong_actor"};
    if (argc > 2) { fprintf(stderr, "Usage: %s [scenario]\n", argv[0]); return 2; }
    if (argc == 2) {
        bool found = false;
        for (size_t i = 0; i < sizeof valid / sizeof valid[0]; ++i)
            if (strcmp(argv[1], valid[i]) == 0) found = true;
        if (!found) { fprintf(stderr, "Unknown scenario: %s\n", argv[1]); return 2; }
        snprintf(scenario, sizeof scenario, "%s", argv[1]);
    }
    printf("=== CNC cell controller, scenario: %s ===\n", scenario);
    double idle_after_reset = 0;
    double recovery_position = 0;
    while (sim_s < 400.0) {
        mock_update();
        step();
        if (state == FAULT && !safe_fault_outputs()) {
            puts("RESULT: FAIL (unsafe fault outputs)"); return 1;
        }
        if (completed_cycles > 0 && state == IDLE) {
            bool ok = expected_fault() == NO_FAULT && first_fault == NO_FAULT &&
                      homed && vel_x == 0 && out_velocity == 0 &&
                      waiting_for == 0 && !out_access1 && !out_access2 &&
                      next_req == 109;
            if (is_scen("stale") || is_scen("wrong_actor")) ok = ok && stale_ignored > 0;
            if (is_scen("pose") || is_scen("home_pose")) ok = ok && pose_blocked_ticks > 0;
            puts(ok ? "RESULT: PASS (cycle complete, stationary in IDLE)" :
                      "RESULT: FAIL (cycle invariants)");
            return ok ? 0 : 1;
        }
        if (ever_faulted) {
            if (state != IDLE || !safe_fault_outputs()) {
                puts("RESULT: FAIL (Reset caused action or unsafe outputs)"); return 1;
            }
            if (idle_after_reset == 0) recovery_position = pos_x;
            if (pos_x != recovery_position) {
                puts("RESULT: FAIL (movement after Reset)"); return 1;
            }
            idle_after_reset += TICK;
            if (idle_after_reset >= 1.0) {
                recovered = true;
                bool ok = first_fault == expected_fault() && first_fault != NO_FAULT;
                if (is_scen("estop") || is_scen("drive") || is_scen("selfcheck") ||
                    is_scen("runaway") || is_scen("home") || is_scen("start_fault"))
                    ok = ok && reset_refused_count > 0;
                puts(ok ? "RESULT: PASS (fault handled; Reset held IDLE without motion)" :
                          "RESULT: FAIL (wrong fault or recovery behaviour)");
                return ok ? 0 : 1;
            }
        }
    }
    printf("RESULT: FAIL (timeout in %s)\n", state_name(state));
    return 1;
}
