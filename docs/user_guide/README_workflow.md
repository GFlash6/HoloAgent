# Demo Mode Runbook

This document describes how to run the repository in **demo mode**, where predefined scripts and known robot workflows are used for demonstrations, validation, and operator-guided execution.

## Scope

Demo mode is intended for:

- scripted demonstrations
- integration verification
- operator-assisted robot execution
- repeatable showcase scenarios

Unlike agentic mode, demo mode does not rely on dynamic skill planning as the primary control path.

## Common Workflow Scripts

- `scripts/intergation/run_holoagent_workflow.sh`
- `scripts/intergation/run_holoagent_running_docker.sh`
- `scripts/intergation/kill_all.sh`
- `scripts/audio/start_audio_ctl_demo.sh`

## Typical Workflow

1. Start the required container or runtime environment.
2. Launch perception, navigation, and robot-side services.
3. Start any required audio or chatbot components.
4. Run the demo launcher script.
5. Monitor logs and stop all processes with the cleanup helper when finished.

## Example Commands

Start the demo stack:

```bash
bash scripts/intergation/run_holoagent_workflow.sh
```

Run the demo stack inside the running Docker environment:

```bash
bash scripts/intergation/run_holoagent_running_docker.sh
```

Stop all integration processes:

```bash
bash scripts/intergation/kill_all.sh
```

## Notes

- Demo scripts are environment-specific and may assume fixed robot IPs, maps, and hardware availability.
- Some demos require audio devices, GPU inference, and robot-side ROS nodes to be available before launch.
- For dynamic skill planning and long-horizon task execution, use agentic mode instead.

## Related Documentation

- [`README.md`](README.md): repository overview
- [`README_agent.md`](README_agent.md): agentic-mode runbook
