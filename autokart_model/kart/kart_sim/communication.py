"""Deterministic safety-message integrity helpers.

CRC-16/CCITT-FALSE is an error-detection code, not authentication.
"""

from __future__ import annotations

from binascii import crc_hqx


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """Return CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, xorout 0)."""
    # CPython delegates crc_hqx to its bounded C implementation.  With the
    # caller-supplied 0xffff initial state this is exactly CRC-16/CCITT-FALSE;
    # keeping the initial argument preserves replay and wire compatibility.
    return crc_hqx(data, initial & 0xFFFF)
