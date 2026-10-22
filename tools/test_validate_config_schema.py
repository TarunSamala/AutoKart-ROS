from validate_config_schema import check_one


def test_passes_for_a_real_schema_version_key(tmp_path):
    f = tmp_path / "x.yaml"
    f.write_text("schema_version: 1\nfoo: bar\n")
    result = check_one({"path": f, "owner": "x", "expected": 1, "kind": "key"})
    assert result["status"] == "PASS" and result["found_schema_version"] == 1


def test_passes_for_a_comment_only_schema_version(tmp_path):
    f = tmp_path / "x.yaml"
    f.write_text("# config_schema_version: 1 -- comment only\nfoo: bar\n")
    result = check_one({"path": f, "owner": "x", "expected": 1, "kind": "comment"})
    assert result["status"] == "PASS"


def test_fails_on_version_mismatch(tmp_path):
    f = tmp_path / "x.yaml"
    f.write_text("schema_version: 2\n")
    result = check_one({"path": f, "owner": "x", "expected": 1, "kind": "key"})
    assert result["status"] == "FAIL" and result["reason"] == "SCHEMA_VERSION_MISMATCH"


def test_fails_when_schema_version_is_absent(tmp_path):
    f = tmp_path / "x.yaml"
    f.write_text("foo: bar\n")
    result = check_one({"path": f, "owner": "x", "expected": 1, "kind": "key"})
    assert result["status"] == "FAIL" and result["reason"] == "SCHEMA_VERSION_NOT_FOUND"


def test_fails_when_file_is_missing(tmp_path):
    result = check_one({"path": tmp_path / "missing.yaml", "owner": "x", "expected": 1, "kind": "key"})
    assert result["status"] == "FAIL" and result["reason"] == "FILE_MISSING"
