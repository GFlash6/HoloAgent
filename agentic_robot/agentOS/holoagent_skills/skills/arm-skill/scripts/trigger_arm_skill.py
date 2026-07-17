#!/usr/bin/env python3
"""Trigger a predefined arm skill over HTTP."""

from __future__ import annotations

import argparse
from urllib import request


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Trigger a predefined arm skill")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Robot service host")
    parser.add_argument("--port", type=int, default=8000,
                        help="Robot service port")
    parser.add_argument("--skill", required=True,
                        help="Arm skill name, for example high_wave")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print request without sending")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    url = f"http://{args.host}:{args.port}/api/arm/{args.skill}"
    print(f"POST {url}")

    if args.dry_run:
        return 0

    req = request.Request(url=url, data=b"", method="POST")
    with request.urlopen(req) as resp:
        body = resp.read().decode("utf-8", errors="replace")
        print(body)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
