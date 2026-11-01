#!/bin/sh
# Build in a temporary directory. Check program exit status, not log substrings.
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
cc=${CC:-cc}
flags='-std=c11 -Wall -Wextra -Wpedantic -Werror'
"$cc" $flags main.c -lm -o "$work/cnc_cell"
"$cc" $flags test_controller.c -lm -o "$work/test_controller"
pass=0
for scenario in normal stale pose estop drive ack home runaway selfcheck home_pose grant grant_loss stuck notready inposition start_fault wrong_actor; do
    if "$work/cnc_cell" "$scenario" > "$work/$scenario.log" 2>&1; then
        printf 'PASS  %s\n' "$scenario"
        pass=$((pass+1))
    else
        cat "$work/$scenario.log"
        printf 'FAIL  %s\n' "$scenario"
        exit 1
    fi
done
if "$work/cnc_cell" unknown > "$work/unknown.log" 2>&1; then
    echo 'FAIL  unknown scenario accepted'
    exit 1
fi
printf 'PASS  invalid CLI scenario rejected\n'
if "$work/test_controller" > "$work/unit.log" 2>&1; then
    tail -n 1 "$work/unit.log"
else
    cat "$work/unit.log"
    exit 1
fi
printf '%s simulation scenarios passed; CLI validation passed; regression assertions passed.\n' "$pass"
