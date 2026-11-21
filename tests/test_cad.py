import pytest

manifold3d = pytest.importorskip("manifold3d")


def test_printed_parts_build_fit_the_printer_and_do_not_clash(tmp_path):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hardware" / "cad"))
    import parts
    rows = parts.main(tmp_path / "stl")
    assert all(fits for *_, fits in rows)
    for wire in ("032", "028"):
        assert parts.clash_check(wire) == []


def test_cad_matches_the_machine_config():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hardware" / "cad"))
    import parts
    from retainer_bender.machine import Machine
    m = Machine.load()
    assert parts.ARM_R == m.arm_radius_mm
    assert parts.PIN_D == m.pin_diameter_mm
    assert parts.DIE_HALF == m.die_halfwidth_mm
    assert parts.ARM_SWEEP > m.arm_max_deg
