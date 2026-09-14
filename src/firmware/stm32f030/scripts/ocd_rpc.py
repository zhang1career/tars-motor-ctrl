#!/usr/bin/env python3
"""Talk to a persistent OpenOCD Tcl RPC port (default 6666).

Each command string is sent in one payload, terminated by 0x1a.
Prints the reply. Exits 2 on connect/timeout so callers can restart.
"""
from __future__ import annotations

import argparse
import socket
import sys


def rpc(cmds: list[str], port: int = 6666, timeout: float = 20.0) -> str:
    payload = "\n".join(cmds)
    s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
    try:
        s.settimeout(timeout)
        s.sendall(payload.encode("utf-8") + b"\x1a")
        data = b""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk
            if b"\x1a" in data:
                break
    finally:
        s.close()
    return data.split(b"\x1a")[0].decode("utf-8", "replace")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=6666)
    p.add_argument("--timeout", type=float, default=20.0)
    p.add_argument("cmds", nargs="+")
    args = p.parse_args()
    try:
        sys.stdout.write(rpc(args.cmds, port=args.port, timeout=args.timeout))
    except (OSError, socket.timeout) as e:
        print(f"ocd_rpc: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
