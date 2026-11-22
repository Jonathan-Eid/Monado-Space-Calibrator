#!/usr/bin/env python3
"""Inspection client for the Linux headless calibration app.

Linux counterpart of scripts/Inspect-App.ps1: speaks the same UTF-8 JSON
protocol over the AF_UNIX socket at $XDG_RUNTIME_DIR/VDXRCalibration.<pid>.sock
(one request, one response, then acknowledge with {"ack":true}).

Usage: inspect-app.py [--pid PID | --socket PATH] [--request JSON]
"""
import argparse
import json
import os
import socket
import sys

ACK = b'{"ack":true}'


def socket_path(pid: int) -> str:
    runtime = os.environ.get("XDG_RUNTIME_DIR") or "/tmp"
    return os.path.join(runtime, f"VDXRCalibration.{pid}.sock")


def inspect(path: str, request: str, timeout: float) -> str:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout)
        client.connect(path)
        client.sendall(request.encode("utf-8"))
        chunks = []
        while True:
            data = client.recv(65536)
            if not data:
                break
            chunks.append(data)
            try:  # a complete JSON document is the whole response
                json.loads(b"".join(chunks).decode("utf-8"))
                break
            except ValueError:
                continue
        client.sendall(ACK)
        return b"".join(chunks).decode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", type=int, default=0)
    parser.add_argument("--socket", default="")
    parser.add_argument("--request", default='{"op":"state"}')
    parser.add_argument("--timeout", type=float, default=7.0)
    args = parser.parse_args()
    json.loads(args.request)  # refuse to send malformed requests
    path = args.socket or socket_path(args.pid)
    if not os.path.exists(path):
        print(f"inspection socket not found: {path}", file=sys.stderr)
        return 1
    print(inspect(path, args.request, args.timeout))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
