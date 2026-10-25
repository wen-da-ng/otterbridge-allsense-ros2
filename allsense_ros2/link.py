"""Where the bridge gets the phone's pairing token. Pure Python (no rclpy), so it's unit-tested anywhere.

ROS parameters are readable by anyone on the DDS domain (``ros2 param get``), so the token is best
kept out of them: in the environment variable ``ALLSENSE_TOKEN`` or in a file named by the
``token_file`` parameter. A token in ``url`` (or the ``token`` parameter) still works; the bridge
then blanks it from the parameters and warns.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Mapping, Optional
from urllib.parse import unquote

TOKEN_ENV = "ALLSENSE_TOKEN"


def strip_token(url: str) -> str:
    """``url`` without its ``token=…`` query parameter."""
    base, sep, rest = url.partition("?")
    if not sep:
        return url
    query, hash_, fragment = rest.partition("#")
    kept = [p for p in query.split("&") if p and not p.startswith("token=")]
    return base + ("?" + "&".join(kept) if kept else "") + hash_ + fragment


def has_token(url: str) -> bool:
    return strip_token(url) != url


@dataclass(frozen=True)
class Link:
    url: str  # the phone's address, without the token (safe to show and to keep as a parameter)
    token: Optional[str]  # None: the url had none and nothing else gave one
    exposed: bool  # the token was in a ROS parameter (url or token), so it should be blanked there


def resolve(url: str, token_param: str = "", token_file: str = "",
            env: Optional[Mapping[str, str]] = None) -> Link:
    """The token from, in order: ``token_file``, ``$ALLSENSE_TOKEN``, the ``token`` parameter, the url.

    Raises ``ValueError`` (without the token in the message) if ``token_file`` can't be read or is empty."""
    env = os.environ if env is None else env
    exposed = bool(token_param) or has_token(url)
    token: Optional[str] = None
    if token_file:
        try:
            with open(os.path.expanduser(token_file), encoding="utf-8") as f:
                token = f.read().strip()
        except OSError as e:
            raise ValueError(f"Can't read token_file {token_file!r}: {e.strerror or e}") from None
        if not token:
            raise ValueError(f"token_file {token_file!r} is empty")
    elif env.get(TOKEN_ENV, "").strip():
        token = env[TOKEN_ENV].strip()
    elif token_param:
        token = token_param
    else:
        for part in url.partition("?")[2].partition("#")[0].split("&"):
            if part.startswith("token="):
                token = unquote(part[len("token="):]) or None
    return Link(strip_token(url), token, exposed)
