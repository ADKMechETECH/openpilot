"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

On-car test console (DEV-TEST BUILD ONLY, branch civic-6mt-dev-test — never promoted or upstreamed).

Telemetry + a steering-wheel button read + a UI. ZERO driving authority: it only reads signals, labels/logs them
and publishes status to a file. It never writes an actuator, a plan value, a cereal message, or any Param other
than its own TestConsole* keys, and it never changes a feature toggle (it only reads them to show "toggle ON?").
"""
