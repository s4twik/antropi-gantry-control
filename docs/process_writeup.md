# My process — Antropi Electronics Assessment

Implementation notes below were corrected after firmware regression testing. The
original choices and development account are retained; current limits are in README.

I started with reading the problem statement.

I asked Claude to make a simple diagram giving the problem statement so I can visualise it better and have something to look back to for reference.

![Reference diagram of the cell, the one I kept coming back to](reference_diagram.png)

(The reference diagram: carriage with the robot sitting at home 0 m, raw and finished trays at 0.3 m, CNC1 at 1.2 m, CNC2 at 2.5 m, end of the rail at 3 m. Every design discussion after this point was basically me pointing at this picture.)

Parameters given:

| Parameter | Given value |
|---|---|
| Gantry travel | 3 m |
| Approximate moving mass | 35 kg |
| Desired travel speed | 1 m/s |
| Field sensor voltage | 24 V DC |
| Home reference | 0 m |
| Tray station | 0.3 m |
| CNC1 station | 1.2 m |
| CNC2 station | 2.5 m |

Okay, now we start with Task 1.

------ Task 1: Choose the control hardware ------

Recommend a suitable motor type, a compatible commercial motor driver, and a controller board. Give a short justification and relevant datasheet links; preliminary motor selection is enough. Include a system block diagram with power supplies, controller, motor drive and motor. Home sensor, end limits, robot/CNC signals, and emergency stop at block level. A small I/O table with purpose, direction, and voltage level. How the controller sends commands to the drive and receives status.

------ Thoughts (MOTOR AND DRIVER) ------

1. The weight of the object that needs to be carried matters the most here. (35 kg)
2. Motor needs to know the position.

SOOOOO, 35 kg, up to 1 m/s, stop repeatably at 0.3, 1.2, and 2.5 m, and never lose track of where it is.

So what are the suitable options here?? Before that I did simple maths (used AI) here:

> For a rough sense of scale, not a final torque calc: accelerating 35 kg at, say, 1 m/s² needs about 35 N before friction and margins, and the actual motor torque then depends on your pulley radius or screw lead.

Let's do some research on motors now....

| Type | Pros | Cons |
|---|---|---|
| 1. Open loop stepper | Cheap, simple | No feedback |
| 2. Closed loop stepper | Cheap, simple, feedback, good at low speed | Not so good at high speed |
| 3. AC/BLDC servo | Encoder feedback, torque holds up at speed | EXPENSIVE |

So, we'll be using a BLDC motor — more precisely, an industrial AC servo, which is the grown-up version of the same idea: a brushless motor with a proper encoder and a matched drive doing the closed loop.

Now moving on to which one.

USED AI FOR RESEARCH AND COMPARISON (BUT WAS LOOKING THROUGH DATASHEETS MYSELF ALSO)

Prompt mentioned: torque range that can plausibly move 35 kg after gearing, encoder feedback as standard, a matched drive you can actually buy, and documentation you can cite.

| Motor / drive family | Pros | Cons | Datasheet |
|---|---|---|---|
| Delta ASDA-A3 | 24-bit absolute encoder; high bandwidth; ASDA-M variant is made for gantry synchronisation; good docs and India support | Mid-range tuning still needed; less prestige than Yaskawa / Mitsubishi | Catalog |
| Yaskawa Sigma-7 / Sigma-X | CNC-proven; 24-bit encoder; excellent tuning; strong India support | Expensive; MECHATROLINK / EtherCAT lock-in; overkill for one axis | Datasheet |
| Mitsubishi MELSERVO-J5 | 26-bit battery-less absolute encoder; stable at high speed; machine-tool proven | Expensive; CC-Link / EtherCAT focused; software learning curve | Datasheet |
| Siemens SIMOTICS + SINAMICS S210 | Deep PLC integration; PROFINET; huge power range; heavy-industry robust | Expensive; heavyweight for one axis; complex commissioning | Datasheet |
| Kollmorgen AKM / AKM2G + AKD | Huge frame / torque range; many feedback, brake, and connector options; low-voltage windings available | Price varies by option; configuration is complex | Datasheet |
| Schneider Lexium 32 + BMH / BSH | Proven in packaging and handling gantries; wide range; CANopen and EtherCAT | Mid to high price; Lexium 32 is an older platform | Datasheet |
| Inovance SV660 / MS1 | Good value; 23-bit encoder; modern drive features; growing support | Newer brand, less long-term track record than Japanese / European brands | Hardware guide |

So, now what's the choice?

https://inno-motion.files.svdcdn.com/production/downloads/02.%20AC%20Servo%20system/01.%20Catalog/Catalogus_ASDA_A3_EP.pdf?dm=1779667688

Motor: ECM-A3L
Drive: ASD-A3-0721-L

WHY???

One rule I fixed in my head first: a servo motor and its drive are a matched pair from one company. You never mix, like, a Yaskawa motor with a Delta drive, it just doesn't work like that. So the choice was really which family, then the pair inside it.

Motor: Delta ECM-A3L, 750 W, 3000 rpm, low inertia. From the catalogue, verified numbers: rated torque 2.39 Nm, peak torque 8.36 Nm, rated current 5.1 A. The axis needs roughly 100 N of force at the carriage (accel + friction, guesstimate), which at 1 m/s is only about 100 W of mechanical power. 750 W is comfortable headroom without going into silly money. If the real transmission maths ever demanded more, the same family steps up to the 1 kW pair, nothing else in the design changes.

Drive: ASD-A3-0721-L, 220 V, pulse-train type. The model number itself decodes in the catalogue: 07 = 750 W, 21 = 220 V, L = pulse-train input. That last letter is the important one, because pulse-train is exactly how I planned to talk to it (more on that below).

------ Thoughts (CONTROLLER) ------

My first choice is "STM32" because I remember from my IoT classes that it's used in industries because of certain certifications (I don't remember what certifications).

But before that, let's think what all do we need to communicate to the board???

- Direction and pulse-train, and obviously C firmware, because we gotta run a mock on PC as well.

Now which STM32??

I looked up "what STM32 controller is generally used for motor drivers?"

- Nucleo-G474RE — Cortex-M4, 170 MHz, STM32G4 family built for motor control.

Then I started researching if there is any functional difference here, in this situation (was using AI for comparison), between G474RE and the generally used STM32 Nucleo-F446RE. And in this use case, there's nothing different that matters.

What do I actually lose with the F446RE? 10 MHz of clock, USB OTG, SDIO, a camera interface — none of which this cell uses. What does the G474RE give extra? Fancy motor-control timers, better analog, CAN-FD — also not needed here, because the drive does the actual motor control loop itself, my board only sends pulses and reads signals. Same flash, same RAM, same 3.3 V logic, same Nucleo-64 format.

So I choose the legacy STM32 Nucleo-F446RE.

Controller: STM32 Nucleo-F446RE, 3.3 V logic
Drive: Delta ASD-A3-0721-L | Motor: Delta ECM-A3L 750 W

------ Thoughts (HOMING SENSOR) ------

Inductive proximity sensor makes sense.

M18 inductive proximity sensor, 3-wire PNP, running at 24 V.

Why not the other options I considered: a mechanical switch wears out, bounces, and its trigger point drifts, and homing accuracy is the one thing this whole machine stands on. A photoelectric sensor hates dust and coolant mist, which a CNC cell has plenty of. An encoder index pulse repeats every revolution, so it can't tell you where on a 3 m rail you are by itself. Inductive it is: no contact, no wear, sealed, cheap, standard.

One detail that came later and matters: home sensor is the normally-OPEN type, end limits are normally-CLOSED type, on purpose. Explained in the sensor circuit section below.

------ Thoughts (I/O TABLE) ------

I start with the basics: signal, pulse, on, off, CNC 1 and 2 handshakes, RESET, FAULTS, limits. Then I used AI and asked it to come up with everything additional it can think of, and also mentioned the need of the optocoupler between the 24 V and 3.3 V line, to keep the controller free from interference.

What I understood properly while making this table: there are THREE voltage worlds in this machine and they should never touch each other directly. 220 V AC is motor power, drive-to-motor only. 24 V DC is the industrial field world, all sensors and handshakes live there because it survives long cable runs and factory noise. 3.3 V is my controller's world, and 24 V would kill an STM32 pin instantly. The optocoupler is the bridge: signal crosses as light, voltage never crosses.

| Signal | Counterpart | Direction | Field voltage | Logic side | Purpose |
|---|---|---|---|---|---|
| Pulse | Delta drive, PT input | Out | 5 V differential via line driver | 3.3 V timer output | Position steps to the drive |
| Direction | Delta drive, DIR input | Out | 5 V differential via line driver | 3.3 V GPIO | Travel direction, + or − along the rail |
| Servo-ON / Enable | Delta drive, Servo-ON | Out | 24 V via opto / transistor | 3.3 V GPIO | Enables permitted homing and normal moves when fault-free |
| Drive Fault / Alarm | Delta drive, Alarm output | In | 24 V via opto | 3.3 V GPIO | Drive fault, stop motion and enter fault state |
| Drive Ready | Delta drive, Ready output | In | 24 V via opto | 3.3 V GPIO | Drive powered and ready to accept commands |
| In-position | Delta drive, Positioning complete | In | 24 V via opto | 3.3 V GPIO | Drive reports commanded position reached |
| Home sensor | Home sensor at 0 m | In | 24 V DC via opto | 3.3 V GPIO | Home position detected during homing |
| Limit minus | End limit, 0 m side | In | 24 V DC via opto | 3.3 V GPIO | Unexpected hit is a fault, normal travel must not reach it |
| Limit plus | End limit, 3 m side | In | 24 V DC via opto | 3.3 V GPIO | Unexpected hit is a fault, normal travel must not reach it |
| Robot travel-pose confirm | UR5 robot | In | 24 V via opto | 3.3 V GPIO | Robot is in safe travel pose, gantry may move |
| Robot request | UR5 robot | Out | 24 V | 3.3 V via transistor | Ask robot to pick, place, or reorient |
| Robot done | UR5 robot | In | 24 V via opto | 3.3 V GPIO | Robot finished the current request |
| CNC1 access grant | CNC1 lathe | In | 24 V via opto | 3.3 V GPIO | Door open and lathe ready for load / unload |
| CNC1 request | CNC1 lathe | Out | 24 V | 3.3 V via transistor | Signal load complete, ask lathe to start machining |
| CNC1 machining done | CNC1 lathe | In | 24 V via opto | 3.3 V GPIO | Lathe finished machining end 1 |
| CNC2 access grant | CNC2 lathe | In | 24 V via opto | 3.3 V GPIO | Door open and lathe ready for load / unload |
| CNC2 request | CNC2 lathe | Out | 24 V | 3.3 V via transistor | Signal load complete, ask lathe to start machining |
| CNC2 machining done | CNC2 lathe | In | 24 V via opto | 3.3 V GPIO | Lathe finished machining end 2 |
| Start | Operator pushbutton | In | 24 V via opto | 3.3 V GPIO | Separate start after reset, starts the cycle |
| Reset | Operator pushbutton | In | 24 V via opto | 3.3 V GPIO | Deliberate fault reset, does not restart motion by itself |
| E-stop status | E-stop circuit | In | 24 V via opto | 3.3 V GPIO | Monitor only, safety itself is the hardware contactor |
| Status beacon | Beacon / LED | Out | 24 V via transistor | 3.3 V GPIO | Running / fault indication |

Original I/O allocation: 15 inputs, 7 outputs, 22 lines. The request/DONE contacts do not implement the message transport; the current allocation and reserved pins are qualified in circuit.pdf. Later I also assigned every single one an actual STM32 pin (pulse on a timer channel, the four nasty safety inputs on interrupts, debug UART and SWD pins deliberately kept free), that table is in circuit.pdf.

Things that are NOT controller I/O, which confused me at first: the encoder (it goes motor-to-drive only, the loop is closed inside the drive), the request ID (message data, requiring a real transport beyond the discrete I/O plan), and the E-stop contactor coil (hardware, below).

------ Thoughts (HOW THE CONTROLLER TALKS TO THE DRIVE) ------

This part I had to think through separately because it's the actual command path.

The drive's pulse-train mode works like a stepper, except closed loop: my timer peripheral spits out pulses, each pulse means "move one fixed small step", and the direction pin says which way. So position control from my side is literally counting. Command 0.9 m worth of pulses, the gantry should be 0.9 m further along, and the drive's own encoder loop makes sure the motor actually gets there even if the load fights it.

Coming back the other way, the drive talks in simple status signals: Ready (I'm alive), In-position (I reached what you commanded), Alarm (something is wrong with me). All 24 V, all through optocouplers into my inputs, and Alarm is one of the four interrupt pins because I want to know THAT one fast.

Options I rejected here: analog ±10 V speed command, because then I'd need a DAC and analog signals pick up noise, and position becomes my problem instead of the drive's. And a fieldbus like EtherCAT, which is genuinely nice but complete overkill for one axis, plus this drive variant (the L) is the pulse-train one anyway.

------ Thoughts (BLOCK DIAGRAM AND THE E-STOP) ------

For the diagram I drew it in layers in my head first: power at one side (220 V AC into the drive, 24 V DC supply for the field, 3.3 V from the Nucleo's regulator for logic), controller in the middle, drive and motor on the motion side, and all the field devices (sensors, robot, both lathes, operator panel) talking to the controller through the interface circuits.

The E-stop thinking is important and I got it wrong in my head initially: the E-stop is NOT a software feature. Its normally-closed contacts sit in a hardware chain that drives a safety contactor, and the contactor physically cuts the 220 V power to the drive. If my firmware hangs, the E-stop still works. My controller only monitors one auxiliary contact so it knows the chain opened and can show a fault. The controller deliberately stays powered during an E-stop, otherwise it couldn't tell anyone what happened.

------ Thoughts (HOMING — WHY THREE STEPS?) ------

This design does not trust an absolute carriage reference at power-on; motor encoder information alone has not been mapped to a validated axis position. So before any normal move, the machine has to go find its zero. That's homing, and it goes:

1. Approach: drive slowly (0.25 m/s) towards minus until the home sensor triggers.
2. Back off: reverse until the sensor clears again, about 0.10 m.
3. Re-approach: creep back at 0.05 m/s. The second trigger edge is the accurate zero, position = 0, homed = true.

And homing itself can fail, which the design has to catch: if the sensor is dead you just keep driving, so there's a travel bound (3.5 m) and a timeout (20 s), and hitting the end limit during homing is an immediate fault. If the sensor is stuck ON, the back-off step never sees it clear, that's a fault too. A dead sensor and a stuck sensor are two different failures and the sequence catches both. No homing, no normal moves, no exceptions — every move state checks homed first.

------ Task 2 area: the sensor circuit ------

Brief wanted one real circuit, so I took the home sensor channel and designed it fully. One channel carries every idea; the limits reuse it.

24 V PNP sensor output (black wire) → 1N4007 diode → 2.2 kΩ resistor → PC817 optocoupler LED → field 0 V.

The original circuit uses a series 1N4007 and 2.2 kOhm, 0.5 W resistor. With assumed 0.7 V diode and 1.2 V LED drops, current is 7.32 mA at 18 V, 10.05 mA at 24 V and 12.77 mA at 30 V. Resistor power reaches 0.359 W at 30 V. Derating, LED reverse clamp/transient protection and worst-case CTR need verification; the series diode is not a complete protection design.

Logic side: PC817 collector to GPIO, emitter to logic ground, 10 kOhm pull-up to 3.3 V and 100 nF to ground. The nominal RC is 1 ms. The submitted mock samples at 50 ms and does not implement software debounce. Home NO detection reads LOW; an NC limit fault or broken wire reads HIGH. At 50 mm/s homing creep, one model tick corresponds to 2.5 mm, not a validated precision specification.

Wire-break behaviour is the part I'm happiest with. Home sensor normally-OPEN: cut its wire and it reads "not home" forever, homing hits its bound and faults — safe, and it can never fake a home. Limit sensors normally-CLOSED: cut a limit wire and it reads "limit hit" — instant safe stop. Same circuit, opposite failure direction, chosen per job. That distinction is the entire reason the two sensor types differ.

Also: no ADC anywhere in this design, deliberately. I got asked "no ADC??" and the answer is everything here is a discrete 24 V on/off signal, the encoder is digital and lives inside the drive's world, and the command is a pulse train. An analog drive mode would have needed a DAC, not an ADC. There is simply nothing analog to read. (One optional extra exists, monitoring the 24 V bus through a divider, purely diagnostics. Not needed, not in the design.)

The KiCad chapter, honestly: I put the schematic into a KiCad file and version 1 taught me two lessons. First it wouldn't even open — I'd drawn the diode/LED triangles as "polygon" elements, which are apparently not a legal shape inside a KiCad schematic symbol, only polylines are. Fixed that, it opened. THEN, from a screenshot, a worse bug: KiCad library coordinates are Y-up, so my asymmetric symbols were silently flipped top-to-bottom in the netlist — sensor power pins swapped, optocoupler anode/cathode and collector/emitter swapped — while the drawing looked perfectly fine. Version 2 was rebuilt with pin positions audited coordinate by coordinate and parts spread out properly. Lesson recorded: a schematic that LOOKS right can still be wrong, verify the pins, not the picture.

------ Task 3 area: the state machine ------

Before code, the whole cycle as a state diagram, because this cycle is exactly a sequence of "do something, wait for confirmation, only then move on".

I drew the state diagram myself first, starting with the 10 steps of the brief, which unfolded into around 23 states (homing needs 3, each lathe visit needs grant-wait, load, machining-wait, grant-wait, unload). Then I cross-checked my drawing against a running simulation visualiser of the same cell that I had, specifically its fault-injection buttons — E-stop, drive fault, dropped acknowledgement, stale acknowledgement, dead home sensor, robot not in travel pose, overtravel. I made sure every one of those errors exists in my diagram with the right behaviour BEFORE writing any code.

Then I handed my diagram to the AI and told it to scrutinize it properly — stand your ground, contest my changes, don't just agree. We went to and fro with it, change by change. Score: I won three, it won three.

Where I was right: access grants for load and unload are separate steps, so the diagram now has separate grant states per lathe visit, with the grant releasing after each robot action. Homing faults deserve explicit arrows from all three homing sub-states, including the stuck-sensor case in back-off. And faults come from every ACTIVE state, not literally every state: in IDLE an E-stop doesn't "fault", there's nothing running to fault, it just refuses Start.

Where I was wrong and it pushed back correctly: my "self check failed" transition at power-on was undefined — I had to narrow it to sensor sanity only (both end limits active at once is physically impossible, that means broken wiring), because E-stop/drive checks at INIT contradicted my own refuses-Start rule in IDLE. My power-loss arrow drawn from one state claimed "from ANY state" in its label, which is not how diagrams work; it now terminates the machine properly and the reboot lands back at INIT. Access grants are maintained permissions, not numbered DONE messages; their wait has its own timeout and their loss during robot entry causes a fault.

Final count: 24 states. Once the argument settled, I asked it to give the final diagram back to me as a clean Mermaid file, which is retained in docs/ with the implementation corrections. And one correction from that whole audit I keep coming back to: robot travel pose missing is a BLOCK, not a fault. The move just holds at zero velocity until the robot confirms its pose. Not everything that stops motion is an error, some of it is the machine politely waiting, and mixing those two up would have faulted the cell every time the robot was slow to fold itself away. Same for waiting for a lathe's access grant. Waits have timeouts (a grant wait that never ends does become a missing-acknowledgement fault), but a wait is not a fault.

The 6 fault causes, final list, from every active state: 1) E-stop chain open, 2) drive fault asserted, 3) unexpected end limit including runaway, 4) missing acknowledgement / timeout, 5) homing timeout or travel bound, 6) a load/unload action requested while not stopped at the station. The revision also checks lost access permission, missing Ready and move settling timeout. On a fault: commanded and ideal mock velocity are zeroed, servo commands disabled, local requests cancelled and homed cleared. This does not validate physical braking or external-machine stopping. Recovery is deliberately boring: Reset is REFUSED while observable E-stop, drive-ready/fault or limit inhibitors remain, Reset alone never restarts anything, Start is a separate press, and since homed was cleared, Start means re-home first. No automatic mid-machining recovery, ever.

------ Task 3: the firmware ------

The brief wants real C that runs on a PC with the hardware mocked, not pseudocode. So: one single C file, no RTOS, no board dependency. The file is in three parts — plain variables, the controller, and the fake hardware at the bottom — but a real Nucleo port still needs GPIO, timer/pulse generation, drive feedback, input filtering and message-transport work.

Scheduling, if you can even call it that: a round-robin superloop. One loop, every 50 ms tick it refreshes the inputs, runs the state machine once, checks if the run is over. Same order, every time, nothing preempts anything. When I asked myself "is this round robin??" — yes, the simplest kind, and it's a feature: worst case reaction to anything is one tick, and everything this machine waits on takes seconds anyway. (And like I said, the real safety doesn't live in this loop at all, the E-stop is hardware.)

The handshake in code is the request-ID idea from the design, and it's like 5 lines of thinking: every request gets the next number (101, 102, ...), a DONE only counts if its actor and number match the one I'm waiting for, anything else gets logged as stale and ignored. The revised check matches actor and ID. A real restart additionally needs session synchronization so an old device reply cannot reuse a new session's number.

First version of the code worked but was honestly overbuilt: structs inside structs, an abstract "actor" system, acceleration-profile motion maths the logic didn't need. So I threw that shape away (kept the logic) and rewrote it super, super simple, comments on basically everything, constant-speed moves, replies as simple countdown timers. If a reviewer can't read the controller top-to-bottom in one sitting, the code is wrong for an assessment like this. Rebuilt it with gcc -std=c11 -Wall -Wextra, zero warnings.

Checking it: the revised script builds the same main.c and runs 17 scenarios. Each scenario verifies its outcome and returns a failing exit code on an incorrect state/output. test_controller.c adds 173 direct assertions, including all-active-state fault checks, held Start through Reset, both homing limits, maintained/lost access grants and matching actor/ID. The normal run reaches CYCLE_DONE at 30.00 s and IDLE at 30.05 s, using requests #101 to #109. Captured results are in docs/test_results.txt.

------ Packaging ------

Last part is unglamorous and where submissions actually die, so: circuit.pdf assembled with the block diagram, the full pin-assigned I/O table, the sensor circuit with its calculations, and a state machine summary. KiCad source, state diagram source and the captured test logs went into docs/. README has the exact build commands, the assumptions, the unfinished bits written honestly (firmware verified on PC mock, not yet run on the physical Nucleo; limits drawn as the same schematic rather than separate sheets), time spent, and a disclosure that AI was used as a research/design/code partner, which it was, all through this document also.

The deliverable is one self-contained ZIP named SatwikShreshtha_Electronics_Assignment.zip, without compiled binaries or installed dependencies. Build/run instructions and tests are included. This package does not claim that the assessment has already been uploaded.

------ If I had more time ------

Complete the hardware adapter, define a supported robot/CNC message link and restart synchronization, validate power/protection/grounding, then test with the physical Nucleo and drive. Constant-speed simulation does not validate acceleration, stopping distance or the machine safety system.

Revision notes: homing backoff now uses distance from first detection; every homing
stage checks travel pose and both limits. The pose wait pauses motion timers. Start
is a fresh edge, consumed outside IDLE. The conceptual E-stop contactor drawing is
not evidence of a validated safety system. Input optocouplers alone do not establish
whole-system isolation.
