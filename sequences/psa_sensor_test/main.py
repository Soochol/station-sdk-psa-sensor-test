#!/usr/bin/env python3
"""
PSA Sensor Test Sequence - CLI Entry Point (SDK 2.0)

This module provides the CLI entry point for running the sequence
as a subprocess from Station Service.

Usage:
    python -m sequences.psa_sensor_test.main --start --config '{"wip_id": "WIP001"}'
    python -m sequences.psa_sensor_test.main --start --dry-run
    python -m sequences.psa_sensor_test.main --stop
"""

import sys
from pathlib import Path

# Ensure the sequence package is importable
# This handles both direct execution and subprocess invocation
_sequence_dir = Path(__file__).parent
if str(_sequence_dir) not in sys.path:
    sys.path.insert(0, str(_sequence_dir))

from sequence import PSASensorTestSequence

if __name__ == "__main__":
    exit(PSASensorTestSequence.run_from_cli())
