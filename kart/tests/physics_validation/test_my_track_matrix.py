from pathlib import Path

from kart_sim.my_track_matrix import Variant, _nominal, run_matrix, write_matrix_csv, write_matrix_summary_md


def test_matrix_runs_and_writes_csv_and_summary(tmp_path: Path) -> None:
    rows = run_matrix(profiles=("shared",), variants=(Variant("nominal", 1, _nominal),))
    assert len(rows) == 1
    assert rows[0]["runs"] == 1

    write_matrix_csv(rows, tmp_path / "matrix.csv")
    write_matrix_summary_md(rows, tmp_path / "SUMMARY.md", assumed_values=["example limit"])

    assert (tmp_path / "matrix.csv").read_text().splitlines()[0].startswith("kart_profile")
    summary_text = (tmp_path / "SUMMARY.md").read_text()
    assert "Limits" in summary_text
    assert "example limit" in summary_text
