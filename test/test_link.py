"""Where the bridge's token comes from (allsense_ros2/link.py: no ROS needed)."""

import pytest

from allsense_ros2 import link

PHONE = "http://10.0.0.2:8765/"


def test_strip_token():
    assert link.strip_token(PHONE + "?token=abc") == PHONE
    assert link.strip_token(PHONE + "?a=1&token=abc&b=2") == PHONE + "?a=1&b=2"
    assert link.strip_token(PHONE) == PHONE and not link.has_token(PHONE)
    assert link.has_token("10.0.0.2:8765/?token=abc")


def test_token_from_the_environment_keeps_parameters_clean():
    got = link.resolve(PHONE, env={"ALLSENSE_TOKEN": " abc\n"})
    assert got == link.Link(PHONE, "abc", exposed=False)


def test_token_from_a_file_wins(tmp_path):
    f = tmp_path / "token"
    f.write_text("fromfile\n", encoding="utf-8")
    assert link.resolve(PHONE, token_file=str(f), env={"ALLSENSE_TOKEN": "env"}).token == "fromfile"
    with pytest.raises(ValueError, match="Can't read token_file"):
        link.resolve(PHONE, token_file=str(tmp_path / "missing"), env={})
    (tmp_path / "empty").write_text("\n", encoding="utf-8")
    with pytest.raises(ValueError, match="is empty"):
        link.resolve(PHONE, token_file=str(tmp_path / "empty"), env={})


def test_token_in_a_parameter_still_works_but_is_flagged():
    got = link.resolve(PHONE + "?token=a%2Bb", env={})
    assert got == link.Link(PHONE, "a+b", exposed=True)
    got = link.resolve(PHONE, token_param="abc", env={})
    assert got.token == "abc" and got.exposed
    # The environment wins, but the url still gets cleaned.
    got = link.resolve(PHONE + "?token=old", env={"ALLSENSE_TOKEN": "new"})
    assert got == link.Link(PHONE, "new", exposed=True)


def test_no_token_anywhere():
    assert link.resolve(PHONE, env={}) == link.Link(PHONE, None, exposed=False)
