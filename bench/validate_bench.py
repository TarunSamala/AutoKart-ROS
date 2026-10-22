"""Host-side static checks for the MicroPython bench firmware tree."""

import ast
import hashlib
from pathlib import Path


BENCH_ROOT = Path(__file__).resolve().parent


def literal_assignments(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    values = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and isinstance(node.value, ast.Constant):
            values[target.id] = node.value.value
    return values


def main():
    errors = []
    python_files = sorted(BENCH_ROOT.rglob("*.py"))
    firmware_files = [
        path
        for path in python_files
        if "archive" not in path.relative_to(BENCH_ROOT).parts
    ]
    hashes = {}

    for path in python_files:
        try:
            values = literal_assignments(path)
        except SyntaxError as exc:
            errors.append(f"syntax error in {path.relative_to(BENCH_ROOT)}: {exc}")
            continue

        if path not in firmware_files:
            continue
        if path.name == Path(__file__).name:
            continue
        if " " in path.name:
            errors.append(f"space in Python filename: {path.relative_to(BENCH_ROOT)}")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in hashes:
            errors.append(
                "exact duplicate firmware: "
                f"{path.relative_to(BENCH_ROOT)} and {hashes[digest]}"
            )
        else:
            hashes[digest] = path.relative_to(BENCH_ROOT)
        if "FIRMWARE_VERSION" not in values:
            errors.append(f"missing FIRMWARE_VERSION: {path.relative_to(BENCH_ROOT)}")

    documented_dirs = [BENCH_ROOT]
    documented_dirs.extend(path for path in BENCH_ROOT.rglob("*") if path.is_dir())
    for directory in documented_dirs:
        for required in ("README.txt", "PINOUT.txt"):
            if not (directory / required).is_file():
                errors.append(
                    f"missing {required}: {directory.relative_to(BENCH_ROOT) or '.'}"
                )

    full_setup = literal_assignments(BENCH_ROOT / "full_setup" / "main.py")
    pin_values = {
        name: value
        for name, value in full_setup.items()
        if name.endswith("_PIN") and isinstance(value, int)
    }
    if len(pin_values.values()) != len(set(pin_values.values())):
        errors.append("duplicate GPIO assignment in full_setup/main.py")

    receiver = literal_assignments(BENCH_ROOT / "estop" / "receiver" / "main.py")
    transmitter = literal_assignments(BENCH_ROOT / "estop" / "transmitter" / "main.py")
    protocol_checks = (
        (receiver.get("FREQUENCY"), transmitter.get("FREQUENCY"), "frequency"),
        (receiver.get("MAGIC0"), transmitter.get("MAGIC0"), "MAGIC0"),
        (receiver.get("MAGIC1"), transmitter.get("MAGIC1"), "MAGIC1"),
        (receiver.get("PROTOCOL_VERSION"), transmitter.get("VERSION"), "version"),
        (
            receiver.get("EXPECTED_TRANSMITTER_ID"),
            transmitter.get("TRANSMITTER_ID"),
            "transmitter ID",
        ),
        (receiver.get("CMD_RUN"), transmitter.get("CMD_RUN"), "RUN command"),
        (receiver.get("CMD_STOP"), transmitter.get("CMD_STOP"), "STOP command"),
    )
    for receiver_value, transmitter_value, label in protocol_checks:
        if receiver_value != transmitter_value:
            errors.append(f"E-stop protocol mismatch: {label}")

    expected_limits = {
        "STEER_LIMIT_DEG": 20.0,
        "BRAKE_LIMIT_DEG": 15.0,
        "THROTTLE_MAX_V": 0.70,
        "LORA_LINK_TIMEOUT_MS": 350,
    }
    for name, expected in expected_limits.items():
        if full_setup.get(name) != expected:
            errors.append(
                f"unexpected full-setup {name}: "
                f"{full_setup.get(name)!r} (expected {expected!r})"
            )

    if errors:
        print("Bench validation FAILED")
        for error in errors:
            print("-", error)
        raise SystemExit(1)

    archived_count = len(python_files) - len(firmware_files)
    print(
        f"Bench validation passed: {len(firmware_files) - 1} active firmware files; "
        f"{archived_count} archived Python files syntax-checked"
    )


if __name__ == "__main__":
    main()
