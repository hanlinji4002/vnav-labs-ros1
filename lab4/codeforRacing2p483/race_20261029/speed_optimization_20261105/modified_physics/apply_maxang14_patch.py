#!/usr/bin/env python3
"""Recreate the 14 rad/s test asset from an unmodified TESSE globalgamemanagers.

The race round in this folder was flown with one simulator field changed:
Unity ``PhysicsManager.m_DefaultMaxAngularSpeed`` (class 55, path id 10), stored
as a float at file offset 25008. The patch sets 7.0 -> 14.0 rad/s, i.e. exactly
four bytes. The patched asset itself is not committed; this script produces it
locally from the official file and verifies both hashes.

Usage:
    python3 apply_maxang14_patch.py <globalgamemanagers> [output]

The default output name is ``globalgamemanagers_maxang14`` next to the input.
The runner swaps this file in for a single run and restores the original copy
afterwards, re-verifying the original SHA-256.
"""
import hashlib
import struct
import sys
from pathlib import Path

OFFSET = 25008
ORIGINAL_SHA = "57c1013b47567a6bacf3fc352f7161b573f430461cf073fcc2c319c0568d7d4a"
PATCHED_SHA = "d6ed730809b598e22517a91871c6ab87512ae336829f5cdd65003a4cd52a509d"


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    source = Path(sys.argv[1])
    target = Path(sys.argv[2]) if len(sys.argv) > 2 else source.with_name(source.name + "_maxang14")

    data = bytearray(source.read_bytes())
    if len(data) < OFFSET + 4:
        raise SystemExit("file is too short to contain the PhysicsManager field")
    found = sha256(source)
    if found != ORIGINAL_SHA:
        raise SystemExit("unexpected source SHA-256: {} (expected the official asset)".format(found))

    value = struct.unpack_from("<f", data, OFFSET)[0]
    if abs(value - 7.0) > 1e-6:
        raise SystemExit("offset {} holds {} rad/s, expected 7.0".format(OFFSET, value))

    struct.pack_into("<f", data, OFFSET, 14.0)
    target.write_bytes(data)

    patched = sha256(target)
    print("source :", source, found)
    print("patched:", target, patched)
    if patched != PATCHED_SHA:
        raise SystemExit("patched SHA-256 differs from the recorded value")
    print("OK: 7.0 -> 14.0 rad/s, 4 bytes changed")


if __name__ == "__main__":
    main()
