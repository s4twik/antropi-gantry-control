# Antropi Robotics — Electronics Engineering Intern Assessment

**Candidate:** Satwik Shreshtha, final-year B.Tech, Electronics & Instrumentation, MIT Manipal
**Task:** control layer for a robotic CNC cell — a UR5 arm on a 3 m gantry serving two CNC
lathes. One rod per cycle: pick from tray → machine first end in CNC1 → unload, flip 180° →
machine second end in CNC2 → unload → place in the finished tray.

## Contents of this ZIP

| Path | What it is |
|---|---|
| `README.md` | This file: build/run steps, design summary, assumptions, disclosure |
| `code/main.c` | The complete Task 3 firmware (single C file, mocked hardware included) |
| `code/test_controller.c` | Direct checks of controller states, permissions, outputs and recovery |
| `code/run_tests.sh` | Builds and runs 17 simulation scenarios and the regression assertions |
| `circuit.pdf` | Supplied system overview, qualified I/O plan, original sensor schematic and corrected notes, transition table and recovery rules |
| `docs/state_diagram.md` | The 24-state state machine in Mermaid source (renders on GitHub / mermaid.live) |
| `docs/state_diagram.svg` | The state machine as a vector SVG (rendered from the Mermaid source above) |
| `docs/block_diagram.svg` | The system block diagram as a vector SVG (rendered from the Mermaid source above) |
| `docs/block_diagram.md` | Updated Mermaid source with explicit power conversion, interfaces and mocked message link |
| `docs/home_sensor_interface_v2.kicad_sch` | KiCad schematic source of the sensor channel drawn in `circuit.pdf` (original KiCad source, unchanged) |
| `docs/changes.md` | Short record of firmware fixes and remaining hardware limits |
| `docs/system_overview.png` | Supplied overview image; interface qualifications are in `circuit.pdf` |
| `docs/test_results.txt` | Captured output: test run, one full normal cycle, one fault + recovery |
| `docs/process_writeup.md` | Original design journal, with implementation corrections noted (embeds `docs/reference_diagram.png`, the cell reference picture used throughout) |

## Selected hardware (Tasks 1–2)

| Role | Part | Why this one |
|---|---|---|
| Controller | STM32 Nucleo-F446RE (Cortex-M4, 180 MHz, 3.3 V) | General-purpose F4 board with hardware timers for the pulse train, GPIO and timer resources for the proposed I/O plan, standard CubeIDE support. No RTOS needed: the control loop is a fixed-order round-robin (cyclic) loop, one pass every 50 ms |
| Servo drive | Delta ASD-A3-0721-L (220 V, pulse-train variant) | Preliminary drive choice; verify the exact motor ordering code and combination before purchase; pulse + direction input up to 4 Mpps, 24-bit encoder resolution, continuous 5.1 Arms |
| Servo motor | Delta ECM-A3L, 750 W, low inertia | Rated torque 2.39 Nm, peak 8.36 Nm, 3000 rpm rated. Moving ~35 kg at up to 1 m/s needs on the order of 100 N of force (accel + friction), i.e. ~100 W at full speed — the 750 W pair has large headroom. The motor family and drive are from the same manufacturer; the exact motor suffix remains to be selected; full check with a chosen pulley/lead-screw is the integrator's, per the brief's scope |
| Home sensor | M18 inductive proximity, 3-wire PNP NO, 10–30 V | Non-contact (no wear/bounce), sealed IP67, standard 24 V industrial part. End limits use the NC variant of the same sensor for fail-safe wire-break behaviour |
| Field interface | PC817 optocoupler channel (one per input) | Bridges 24 V field signals to 3.3 V logic with ~5 kV isolation; field and logic grounds never join. Full circuit and calculations in `circuit.pdf` |

## How to build and run the code (Task 3)

Requirements: any C11 compiler. Tested with gcc on Linux. No libraries beyond the
C standard library and libm. No hardware is needed — the hardware is mocked inside
the same file (Part 3 of `main.c`). This is a PC sequencing example, not a
finished Nucleo port. The board adapter and real communication link remain to be built.

```bash
cd code
gcc -std=c11 -Wall -Wextra -o cnc_cell main.c -lm
./cnc_cell            # one full, fault-free cycle
```

Expected: homing runs first (approach, back-off, re-approach), then the cycle states
in brief order with requests #101–#109, ending with
`RESULT: PASS (cycle complete, stationary in IDLE)` at 30.05 s of simulated time.
The sequence reaches CYCLE_DONE at 30.00 s; the following tick returns to IDLE.

Fault and edge-case scenarios (each also ends in a `RESULT: PASS` line when the
behaviour is correct):

```bash
./cnc_cell stale      # a DONE with an old request ID arrives: logged, ignored, cycle completes
./cnc_cell pose       # robot travel pose missing: the move waits at zero velocity, then completes
./cnc_cell estop      # E-stop during a move: FAULT; Reset refused while still pressed; then recovery
./cnc_cell drive      # drive fault: FAULT; Reset refused while fault active; then recovery
./cnc_cell ack        # one reply never arrives: ack-timeout FAULT, then recovery
./cnc_cell home       # home sensor dead: homing FAULT, then recovery
./cnc_cell runaway    # gantry overtravels: unexpected-end-limit FAULT, then recovery
./cnc_cell selfcheck  # both end limits active at power-on: wiring FAULT at INIT
```

Run everything at once:

```bash
sh run_tests.sh       # builds in a temporary folder; checks exit status and assertions
```

The test script runs all 17 named scenarios, rejects an unknown scenario name,
and runs 173 direct assertions from `test_controller.c`. It needs a POSIX shell
and a C11 compiler (`CC=gcc` or `CC=clang` may be supplied). It does not leave
binaries in the submission directory. Individual runs also check their state
and output invariants before printing PASS.

Additional scenarios: `home_pose`, `grant`, `grant_loss`, `stuck`, `notready`,
`inposition`, `start_fault`, and `wrong_actor`.

## Design summary

- **State machine:** 24 states, listed in `docs/state_diagram.md`; the C enum uses the
  identical names, so code and diagram check against each other directly.
- **Homing:** slow approach at 0.25 m/s to the home sensor, back off 0.10 m measured from first detection, re-approach
  at 0.05 m/s; the second edge is position 0. Homing is bounded by a 20 s timeout and a
  3.5 m travel bound; no normal move is possible before `homed` is true.
  Travel-pose waiting pauses homing/move timers and holds zero velocity. Both end
  limits remain checked. A sensor still active after 0.15 m backoff faults.
- **Handshakes:** every robot/CNC action request carries an actor, command and ID. A DONE
  counts only if actor and ID match the pending action. The mock passes these as
  messages, not single electrical bits. CNC access is a separate maintained
  request/grant: held through load/unload and released after robot retreat completes.
  Grant loss during either operation causes FAULT.
- **Interlocks:** a load/unload request is issued only while the gantry is stopped at the
  correct station with Ready and In-position confirmed; checks continue while
  the action is pending. Every move, including homing, runs only while
  the robot confirms travel pose — a missing pose blocks the move, it is not a fault.
- **Faults** (from every active state): E-stop open, drive fault, unexpected end limit,
  missing acknowledgement (ack or grant timeout), homing timeout/travel bound, and an
  action requested while not stopped at the station. Missing Drive Ready, lost
  access grant and motion/in-position timeout are also checked. Entry sets commanded and mock velocity to zero, disables
  servo commands, withdraws access requests, clears pending messages/timers and
  clears `homed`. The mock assumes an immediate stop; this is not a stopping-distance
  or external-machine-stop demonstration. In IDLE, E-stop/drive-fault only refuse Start.
- **Recovery:** Reset checks E-stop, drive fault, Drive Ready and both limits. An accepted
  Reset returns to IDLE with no motion. Start is edge-triggered, consumed outside
  IDLE, and ignored while Reset is held. A fresh press after Reset re-homes first.
  A reboot clears homing and local pending work. A real link also needs session
  resynchronization; repeating numeric IDs across a power cycle is not sufficient.

## Assumptions

1. One workpiece in the cell at a time; the cycle finishes before the next starts.
2. The brief gives one tray position (0.3 m), so the raw and finished trays are treated
   as side-by-side pick/place points inside the 0.3 m tray station zone.
3. All positions are measured along the rail from home = 0 m. Station positions are the
   brief's: tray 0.3 m, CNC1 1.2 m, CNC2 2.5 m; mock end limits at −0.15 m / 3.15 m. These assume clearance
   beyond the 0–3 m working range, not a verified mechanical dimension. Confirm
   available overtravel before mapping this model to a physical rail.
4. The gantry waits at the lathe during machining (it holds the part's station, matching
   the grant/unload sequence), rather than serving the other lathe in parallel.
5. Robot pick / place / flip routines already exist, as the brief states; the controller
   only issues numbered messages and waits for pose-confirmed and DONE. A robot
   load/unload DONE means the robot has completed its routine and retreated clear;
   machine-side interlocks must independently prevent unsafe door/spindle operation.
6. The ~100 N force estimate for the 35 kg axis is an order-of-magnitude sizing check
   only; final transmission (belt/pulley or lead screw) selection and its torque
   arithmetic are outside the brief's scope, like the other excluded mechanics topics.
7. Field signals are 24 V DC as stated in the brief; the operator panel buttons reach
   the controller through the same optocoupler interface as the other field inputs.
8. The E-stop is represented as independent safety hardware. The controller only
   monitors status. Removing mains is the conceptual block shown, not a validated
   stop architecture: actual stopping behaviour, contactor monitoring and device
   safety compatibility remain outside this PC demonstration.

## Time spent

Approximately 14 hours spread over 25–30 October 2026: ~4 h hardware research and
selection, ~2 h block diagram and I/O design, ~3 h sensor circuit and KiCad schematic,
~2 h state machine design and review, ~3 h firmware, fault scenarios and packaging.

## What is unfinished / limitations

- This code runs only on a PC mock. No physical Nucleo, servo drive, robot or CNC was
  used. Pulse generation, scaling, acceleration/deceleration, electrical output
  stages and asynchronous drive feedback remain hardware-port work.
- The message adapter passes `(actor, command, request_id)` and echoes
  `(actor, request_id)` on completion. The exact robot/CNC transport is **not
  implemented**. A single 24 V request and DONE pair cannot carry this information.
  The I/O table reserves those contacts as legacy signals; choose a supported
  message link and define abort, session reset and reconnect behaviour before use.
- Access request/grant is maintained separately. Withdrawing a request on FAULT
  does not prove that a real robot or CNC stopped; external stop coordination and
  manual inspection of an interrupted workpiece remain necessary.
- Reset checks observable inhibitors, not whether every underlying fault is repaired.
  A dead NO home sensor or failed remote actor needs inspection and a bounded retry.
  The mock's service event explicitly clears its injected fault and, where needed,
  manually relocates the modeled carriage. Reset itself never moves it.
- Sampling is 50 ms. The mock has no 1 ms input task or software debounce. The
  proposed 10 kOhm / 100 nF hardware RC is nominally 1 ms; real filtering needs
  implementation and validation. At 50 mm/s creep, one model tick is 2.5 mm;
  submillimetre homing accuracy is not claimed.
- `pos_x` is an ideal simulated position, not a drive encoder measurement. The
  real controller needs commanded pulse counting plus separately read drive
  feedback. Mock Ready means drive healthy independently of Servo-ON; verify the
  selected drive's actual Ready semantics in the hardware adapter.
- The original KiCad input circuit is retained, not electrically validated. Verify
  optocoupler CTR, resistor derating, reverse/transient protection and grounding.
  The series diode alone is not presented as a complete industrial protection design.
- The reference overview is retained for readability. Its general active-LOW label
  applies to NO home detection, not NC limit faults; see the polarity table in
  `circuit.pdf`. It omits some interface and power stages that the updated Mermaid
  source makes explicit. None of these diagrams are fabrication-ready wiring drawings.

## Disclosure of AI assistance

AI assistance was used for research, diagrams, drafting and code review. ChatGPT
assisted with this revision's firmware fixes, regression tests and documentation
corrections. The recorded results were produced by compiling and running the code
included in this ZIP; they are simulation results, not hardware validation.

## References

- Delta ASDA-A3 series catalogue (ASD-A3-0721-L drive and ECM-A3L motor specifications,
  ordering/pairing table): https://www.pneu-hyd.co.th/images/Products/automation/servo/delta/asda-a3/ASDA-A3.pdf
- STMicroelectronics, Nucleo-F446RE board: https://www.st.com/en/evaluation-tools/nucleo-f446re.html
- STMicroelectronics, STM32F446RE microcontroller: https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html
- PC817 optocoupler datasheet, Sharp Microelectronics.
- 1N4007 rectifier datasheet, onsemi.
- Assessment brief: Antropi Robotics, Electronics Engineering Intern Assessment,
  issued 25 October 2026.
