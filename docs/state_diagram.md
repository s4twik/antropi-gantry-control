# State machine — final version

24 states. The C enum in `code/main.c` uses these exact names.
Conventions: one representative arrow is drawn where a rule applies to many
states (stale DONE, power loss); the notes state the rule in full.

```mermaid
stateDiagram-v2
    [*] --> INIT
    INIT --> IDLE : self check passed, homed = false
    INIT --> FAULT : self check failed, sensor sanity

    IDLE --> HOMING_APPROACH : fresh Start, healthy inputs, not homed
    IDLE --> MOVE_TRAY : fresh Start, healthy inputs, homed

    HOMING_APPROACH --> HOMING_BACKOFF : home sensor triggers
    HOMING_BACKOFF --> HOMING_REAPPROACH : sensor clear, relative backoff at least 0.10 m
    HOMING_REAPPROACH --> MOVE_TRAY : sensor triggers again, position set to 0, homed true
    HOMING_APPROACH --> FAULT : homing timeout or travel bound
    HOMING_BACKOFF --> FAULT : homing timeout or travel bound, sensor stuck on
    HOMING_REAPPROACH --> FAULT : homing timeout or travel bound

    MOVE_TRAY --> ROBOT_PICK : arrived tray 0.3 m, stopped, in position
    ROBOT_PICK --> ROBOT_PICK : stale DONE, old id ignored, keep waiting
    ROBOT_PICK --> MOVE_CNC1 : robot done, matching actor and request id
    ROBOT_PICK --> FAULT : ack timeout

    MOVE_CNC1 --> CNC1_GRANT_LOAD : arrived CNC1 1.2 m, stopped, in position
    CNC1_GRANT_LOAD --> ROBOT_LOAD1 : CNC1 access granted
    CNC1_GRANT_LOAD --> FAULT : grant timeout, counts as missing ack
    ROBOT_LOAD1 --> WAIT_CNC1 : robot done, matching actor and id, new request id issued for machining, access released
    WAIT_CNC1 --> CNC1_GRANT_UNLOAD : CNC1 done, matching actor and id
    CNC1_GRANT_UNLOAD --> ROBOT_UNLOAD1 : CNC1 access granted
    CNC1_GRANT_UNLOAD --> FAULT : grant timeout, counts as missing ack
    ROBOT_UNLOAD1 --> ROBOT_FLIP : robot done, matching actor and id, access released
    ROBOT_FLIP --> MOVE_CNC2 : matching robot DONE, rod flipped 180

    MOVE_CNC2 --> CNC2_GRANT_LOAD : arrived CNC2 2.5 m, stopped, in position
    CNC2_GRANT_LOAD --> ROBOT_LOAD2 : CNC2 access granted
    CNC2_GRANT_LOAD --> FAULT : grant timeout, counts as missing ack
    ROBOT_LOAD2 --> WAIT_CNC2 : robot done, matching actor and id, new request id issued for machining, access released
    WAIT_CNC2 --> CNC2_GRANT_UNLOAD : CNC2 done, matching actor and id
    CNC2_GRANT_UNLOAD --> ROBOT_UNLOAD2 : CNC2 access granted
    CNC2_GRANT_UNLOAD --> FAULT : grant timeout, counts as missing ack
    ROBOT_UNLOAD2 --> MOVE_OUTPUT : robot done, matching actor and id, access released

    MOVE_OUTPUT --> ROBOT_PLACE : arrived tray 0.3 m, stopped, in position
    ROBOT_PLACE --> CYCLE_DONE : robot done, matching actor and id
    CYCLE_DONE --> IDLE : homed stays true

    MOVE_CNC1 --> FAULT : unexpected end limit or drive fault
    WAIT_CNC1 --> FAULT : ack timeout, ack dropped
    WAIT_CNC2 --> FAULT : ack timeout, ack dropped
    ROBOT_LOAD1 --> FAULT : action requested while moving or off station
    MOVE_CNC2 --> [*] : power loss, machine terminates

    FAULT --> IDLE : fresh Reset, observable inhibitors clear, homed false

    note right of INIT
        Self check is sensor sanity only:
        LIM_MIN and LIM_MAX active at the same
        time can only be a wiring fault.
        E-stop or drive fault at INIT do not fault
        here, they refuse Start from IDLE instead.
        Power loss from ANY state terminates the
        machine, the arrow is shown once from
        MOVE_CNC2. On power restore the controller
        boots into INIT with homed = false.
    end note

    note right of FAULT
        FAULT is entered from every ACTIVE state
        (homing, moving, grant, robot, wait) on any of
        1 E-stop chain open
        2 Drive fault asserted or Ready missing
        3 Unexpected end limit, including overtravel runaway
        4 Missing acknowledgement, ack timeout,
          including CNC access grant timeout
        5 Homing timeout or travel bound,
          including home sensor dead or stuck on
        6 Action requested or pending off station
        7 Access grant lost during load or unload
        8 Move or in-position timeout
        On entry: drive disabled, motion stopped, requests
        cancelled, homed set false.
        In IDLE and CYCLE_DONE an E-stop or drive fault
        does not fault, it only refuses Start.
        Reset is REFUSED if E-stop is still pressed
        or drive fault is active, Ready missing,
        or either end limit remains active.
        Reset alone never restarts motion.
    end note

    note left of HOMING_APPROACH
        MOVE states first check homed is true.
        In ALL moves, including homing, missing
        robot travel pose means motion is BLOCKED at zero velocity and waits,
        it is not a fault. Move and homing timers
        pause; motion resumes when pose is confirmed.
        Start edges are consumed outside IDLE,
        so held or old Start cannot follow Reset,
        and refused if E-stop or drive fault is active.
    end note

    note left of WAIT_CNC1
        ROBOT and WAIT states accept DONE only
        when actor and ID match the pending job.
        GRANT is a maintained permission, not an ID.
        It stays asserted through load and unload.
        Shown once on ROBOT_PICK as the example.
        Machining waits use a longer timeout
        than robot actions.
    end note
```

The PC run returns from CYCLE_DONE to IDLE, then exits after checking its outputs.
FAULT cancels local mock timers; stopping real external machines requires a separate
validated interface. Power-on must also resynchronize the external message session.
