"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

On-car test console (DEV-TEST BUILD ONLY: lives on the dev-test overlay branch that tracks production — never promoted
or upstreamed).

A vehicle-agnostic core (this package's modules: capture state machine, detectors/analyzers, the registry/session
engine, file IPC, the start button's gestures; the UI framework in selfdrive/ui/sunnypilot/mici) plus one vehicle
profile per car (profiles/: signal map, powertrain, button map, test plan, pass rules, UI allowlist), selected by
fingerprint (profile.py). No profile for a car -> the console is inactive.

Telemetry + a steering-wheel button read + a UI. ZERO driving authority: it only reads signals, labels/logs them
and publishes status to a file. It never writes an actuator, a plan value, a cereal message, or any Param other
than its own TestConsole* keys, and it never changes a feature toggle (it only reads them to show "toggle ON?").
"""
