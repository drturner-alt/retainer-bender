"""Wire sizes the bender is set up for."""

WIRES = {
    "026": {"label": '0.026" SS', "d_in": 0.026, "d_mm": 0.026 * 25.4},
    "028": {"label": '0.028" SS', "d_in": 0.028, "d_mm": 0.028 * 25.4},
    "032": {"label": '0.032" SS', "d_in": 0.032, "d_mm": 0.032 * 25.4},
}


def get_wire(key):
    k = str(key).strip().strip('"')
    if k.startswith("0."):
        k = k[2:]
    k = k.lstrip(".").zfill(3)
    if k not in WIRES:
        raise SystemExit(f"unknown wire {key!r}; choose one of {', '.join(WIRES)}")
    return dict(WIRES[k], key=k)
