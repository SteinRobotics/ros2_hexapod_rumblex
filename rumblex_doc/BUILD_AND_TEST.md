# Resource-limited builds and tests

Use [BUILD.md](../BUILD.md) for terminal sourcing and colcon build/test commands.
This guide defines resource controls and recovery procedures. Run from the
workspace root, `~/Workspace/colcon_rumblex`.

## Access to memory controls (both architectures)

Use the logged-in user's systemd manager, without `sudo`. Check access first:

```bash
id -u
printf '%s\n' "$XDG_RUNTIME_DIR" "$DBUS_SESSION_BUS_ADDRESS"
systemctl --user show-environment >/dev/null
stat -fc %T /sys/fs/cgroup
```

The last command should report `cgroup2fs`. A normal local/SSH login should supply
the user session and its bus. If user systemd access also fails in a normal
terminal, fix the login/session setup before running a heavy job.

**Codex sessions:** on the amd64 development machine, the sandbox cannot access
the user bus even though `/run/user/1000/bus` exists and the environment is correct.
The error was `Failed to connect to user scope bus via local transport: No data available`.
Use the shell tool's `sandbox_permissions: "require_escalated"` for the bounded
`systemd-run` invocation and systemd inspection commands. This successfully used
the existing user manager; it did not require root, new groups, or disabling the
sandbox globally. The workspace's `build`, `install`, and `log` directories also
lie outside this repository's writable sandbox root. Describe the bounded action
in the escalation justification. If escalation is rejected, report it and do not
fall back to an unrestricted build. Do not assume a Pi has UID 1000 or copy another
user's bus address.

## amd64 development machine (approximately 30 GiB RAM)

```bash
export RUMBLEX_MEMORY_HIGH=6G
export RUMBLEX_MEMORY_MAX=8G
export RUMBLEX_MEMORY_SWAP_MAX=1G
export RUMBLEX_BUILD_WORKERS=2
```

These are shared workload limits, not per-compiler allowances. Use one worker
for the smaller recovery batches below. The library and ten affected brain test
suites were successfully built with one worker under these memory limits.

## Raspberry Pi 5 (64-bit Ubuntu / aarch64)

Inspect the actual board rather than assuming its RAM size:

```bash
uname -m
free -h
```

Start with one compiler worker and RAM-relative limits so smaller boards retain
memory for the OS and robot services:

```bash
export RUMBLEX_MEMORY_HIGH=40%
export RUMBLEX_MEMORY_MAX=50%
export RUMBLEX_MEMORY_SWAP_MAX=512M
export RUMBLEX_BUILD_WORKERS=1
```

These are conservative starting recommendations, **not measurements from a Pi**.
Percentages refer to physical RAM. A small-RAM Pi may not fit even one C++
compilation unit inside this budget; use a suitable build machine or diagnose the
specific target instead of automatically increasing limits or adding swap. A swap
cap does not allocate swap. Keep memory-pressure protection enabled. Do not start
a build while simulations, CAD jobs, or another build/test workload are active.

## Shared bounded command wrapper

After selecting the architecture's settings, define this Bash function in the
same shell as the commands below. In Codex, include the definition/settings in
the escalated shell invocation; separate tool calls do not preserve shell functions.

```bash
rumblex_limited() {
    local rumblex_unit="rumblex-check-$(date +%s%N)"
    printf 'Resource scope: %s.scope\n' "$rumblex_unit"
    flock --nonblock "/tmp/rumblex-heavy-$(id -u).lock" \
      systemd-run --user --scope --unit="$rumblex_unit" \
      -p "MemoryHigh=$RUMBLEX_MEMORY_HIGH" \
      -p "MemoryMax=$RUMBLEX_MEMORY_MAX" \
      -p "MemorySwapMax=$RUMBLEX_MEMORY_SWAP_MAX" \
      timeout --kill-after=10s 300s \
      env MAKEFLAGS="-j$RUMBLEX_BUILD_WORKERS -l$RUMBLEX_BUILD_WORKERS" \
          CMAKE_BUILD_PARALLEL_LEVEL="$RUMBLEX_BUILD_WORKERS" \
          CTEST_PARALLEL_LEVEL=1 "$@"
}
```

Use this wrapper for builds, tests, and other heavy jobs across sessions. Its
nonblocking lock prevents cooperating sessions for the same Unix user from
running heavy jobs together. A lock failure means another job holds the budget;
wait for it, do not delete the lock file. Other users or jobs not using the wrapper
are not covered: inspect processes before starting, including their command lines.

```bash
ps -eo user,pid,ppid,comm,args | rg 'colcon|cc1plus|ninja|ctest|pytest|[F]reeCAD|[g]z sim'
```

**Set both worker controls.** In the installed colcon-cmake version, setting only
`CMAKE_BUILD_PARALLEL_LEVEL=2` still resulted in `cmake --build ... -- -j20 -l20`.
The memory-pressure daemon stopped that build. `MAKEFLAGS=-j2 -l2` prevents
colcon from adding its CPU-count-based Make arguments. `--executor sequential`
limits concurrent packages, not compiler processes within a package.

Before a real workload, verify controls using a short scope (the same escalation
is needed from Codex):

```bash
rumblex_limited bash -c '
  cg=$(sed -n "s/^0:://p" /proc/self/cgroup)
  for key in memory.high memory.max memory.swap.max; do
    printf "%s: " "$key"
    cat "/sys/fs/cgroup$cg/$key" || exit 1
  done
'
```

Check that these are finite and match the selected limits (percentages resolve
to bytes). Do not proceed if controls are unavailable or unverified. During a job,
inspect its printed scope from a second terminal:

```bash
systemctl --user show <printed-scope>.scope \
  -p ActiveState -p MemoryHigh -p MemoryMax -p MemorySwapMax -p MemoryCurrent -p MemoryPeak
```

## Build and test commands

Follow [BUILD.md](../BUILD.md#initial-builds) for initial and incremental colcon
builds, package selections, and tests. Define and verify the wrapper above first.
Larger builds may exceed a five-minute batch; inspect logs before retrying and
reduce to focused targets after an interruption.

For focused checks after CMake configuration, select only the targets needed:

```bash
export RUMBLEX_BUILD_WORKERS=1
rumblex_limited cmake --build build/rumblex_brain --parallel 1 \
  --target test_movement_types test_single_pose_target test_sequence_parser
rumblex_limited ctest --test-dir build/rumblex_brain --parallel 1 \
  --output-on-failure -R '^test_(movement_types|single_pose_target|sequence_parser)$'
```

A direct CMake target build does not install the package or rebuild unrelated test
binaries. Only run test binaries rebuilt against the current library. Complete
installation with a bounded colcon build when needed. CTest output reports suites;
GoogleTest XML under `build/rumblex_brain/test_results` reports individual tests.

## Interruptions and recovery

If a tool session disappears or returns an unknown process ID, inspect processes
and the scope before retrying. Stop only the current task's surviving scope with
`systemctl --user stop <printed-scope>.scope`, then verify that its processes exited.
Do not infer completion from a missing tool session.

For a timeout, kill, or memory-pressure interruption, inspect:

```bash
systemctl --user show <printed-scope>.scope -p ActiveState -p Result -p MemoryPeak
journalctl --user -u <printed-scope>.scope -n 30 --no-pager
```

Use `log/latest_build/rumblex_brain/{stdout,stderr,command}.log` for colcon,
`build/rumblex_brain/Testing/Temporary` and `test_results` for tests, and the tool's
captured output for direct CMake commands. Finished scopes may be unloaded; use
the journal if their properties are no longer available.

After inspecting the expensive operation, reduce to one worker and selected
CMake targets or individual tests in separate bounded batches. Keep the limits;
do not disable systemd-oomd or automatically rerun the whole workload. A completed
focused batch is not a full-package build/test result. Report exactly which
checks passed, failed, or were interrupted.

## References

- [systemd resource control](https://www.freedesktop.org/software/systemd/man/latest/systemd.resource-control.html)
- [colcon executor arguments](https://colcon.readthedocs.io/en/released/reference/executor-arguments.html)
- [colcon-cmake build implementation](https://github.com/colcon/colcon-cmake/blob/master/colcon_cmake/task/cmake/build.py)
