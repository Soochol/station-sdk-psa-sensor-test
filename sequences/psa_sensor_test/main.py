"""CLI entry point for PSA Sensor Test sequence."""

from .sequence import PSASensorTestSequence

if __name__ == "__main__":
    exit(PSASensorTestSequence.run_from_cli())
