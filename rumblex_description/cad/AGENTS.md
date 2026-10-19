# RumbleX CAD execution rules

These instructions apply to this directory and all subdirectories.

- Use the CAD virtual environment's .venv/bin/python from this directory.
- Run only one heavy CAD workload at a time, including geometry tests, complete assembly builds, STEP/STL/DXF exports, and nesting jobs. Check for existing CAD Python processes, including those started by other Codex sessions, before launching a job. Do not terminate another session's processes.
- Always wrap heavy CAD commands in verified OS memory controls and a timeout. From this directory, the default pattern is:

```bash
systemd-run --user --scope \
  -p MemoryHigh=6G \
  -p MemoryMax=8G \
  -p MemorySwapMax=1G \
  timeout --kill-after=10s 5m \
  .venv/bin/python -m unittest discover \
  -s tests -p test_nira_lidar.py -v
```

Replace the Python arguments for the focused test or exporter needed. The scope bounds the workload and its children together. These conservative limits assume the approximately 30 GiB development machine; inspect available memory first and lower the budget when needed. Do not run unbounded if systemd user scopes or memory controls are unavailable.

- Select tests relevant to the changed geometry before considering the full suite. Run expensive test modules in separate bounded processes so native CAD allocations are released between modules.
- A lost tool session is not proof that a CAD process exited. Inspect its process command line and state before retrying; confirm the original task-owned process and children have exited. Never launch a duplicate merely because polling reports an unknown process ID.
- After a timeout, memory-limit failure, or unexplained growth, preserve available logs, identify the expensive test or geometry operation, and investigate it before another full run. Do not automatically raise limits or repeat the same failing command. Any justified adjustment must retain explicit memory and time bounds and leave room for the desktop and other applications.
- Treat concurrent Codex sessions as sharing one machine-wide resource budget. Sequential execution is the default; any necessary concurrent heavy work must share an enforced aggregate memory budget.
- Report which checks passed, failed, or were stopped by resource limits. Never describe a partial or interrupted CAD suite as passing.
