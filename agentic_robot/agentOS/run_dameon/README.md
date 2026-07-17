# Background Daemon Helper

This directory contains the helper used to keep robot-side background processes running for longer-lived HoloAgent workflows.

## Files

```text
run_dameon/
├── README.md
└── run_g1_background_daemon.py
```

## Usage

```bash
python3 agentic_robot/agentOS/run_dameon/run_g1_background_daemon.py
```

## Purpose

Use this helper when a robot-side process must remain alive independently from an interactive shell session or a short-lived launcher.

## Notes

- The directory name `run_dameon` is preserved to match the current repository layout.
- Review the Python script before deployment to confirm environment assumptions and process behavior.
