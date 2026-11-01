# Revision notes

The original 24-state sequence, component choices and single-file C layout remain.

- Start and Reset use sampled edges. Start is consumed outside IDLE and a held Start
  cannot restart the cell after Reset. Reset checks both limits and drive readiness.
- Fault entry explicitly stops commanded and mock motion, disables servo commands,
  clears pending requests/replies and withdraws access requests.
- All homing stages check travel pose and both limits. Backoff is measured relative
  to first detection. Normal moves require homing and wait for In-position.
- CNC access is requested explicitly and maintained throughout load/unload.
  Loss of access faults; release follows completion of the robot retreat routine.
- Action completion matches actor and request ID. The message transport remains a
  mock and is not falsely mapped onto a single 24 V request/DONE pair.
- The test script checks process exit status and runs direct controller assertions.
  It also rejects an unrecognized scenario instead of silently running normally.
- CYCLE_DONE now returns to stationary IDLE. The one-cycle command-line run exits there.
- Documentation timing, polarity, state count and hardware-port claims were corrected.
  The supplied overview and original sensor schematic were retained with qualifications.

## Boundaries

This is a runnable sequencing demonstration. No real hardware has been tested.
The original sensor circuit is unchanged; electrical protection, device compatibility,
real stopping behaviour and the robot/CNC communication link still need validation.
