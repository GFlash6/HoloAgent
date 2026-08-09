#!/usr/bin/env python3
"""Send a semantic navigation request over HTTP."""

from __future__ import annotations

import argparse
import json
from urllib import request


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send a semantic navigation request")
    parser.add_argument("--robot-url", default="http://127.0.0.1:8000",
                        help="Robot service base URL")
    parser.add_argument("--cmd", required=True,
                        help="Semantic target as floor,room,object")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print request without sending")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    payload = {"cmd": args.cmd}
    url = f"{args.robot_url.rstrip('/')}/api/semantic_nav"

    print(f"POST {url}")
    print(json.dumps(payload, ensure_ascii=False, indent=2))

    if args.dry_run:
        return 0

    req = request.Request(
        url=url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with request.urlopen(req) as resp:
        body = resp.read().decode("utf-8", errors="replace")
        print(body)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
