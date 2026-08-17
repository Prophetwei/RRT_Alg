#!/usr/bin/env bash
set -e

echo "=== irun runtime args: $* ==="
exec irun TESTBED.sv \
    -notimingchecks \
    -loadpli1 debpli:novas_pli_boot \
    -access +wrc \
    -define RTL_TOP \
    "$@"
