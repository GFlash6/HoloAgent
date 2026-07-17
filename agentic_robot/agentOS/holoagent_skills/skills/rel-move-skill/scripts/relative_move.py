#!/usr/bin/env python3
"""Send a relative movement request over HTTP."""

from __future__ import annotations

import argparse
import json
from urllib import request


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send a relative movement request")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Robot service host")
    parser.add_argument("--port", type=int, default=8000,
                        help="Robot service port")
    parser.add_argument(
        "--forward",
        type=float,
        default=0.0,
        help="Target relative forward displacement in meters",
    )
    parser.add_argument(
        "--left",
        type=float,
        default=0.0,
        help="Target relative left displacement in meters",
    )
    parser.add_argument(
        "--rotation",
        type=float,
        default=0.0,
        help="Target heading rotation relative to current heading in degrees",
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Print request without sending")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    payload = {
        "forward": args.forward,
        "left": args.left,
        "rotation": args.rotation,
    }
    url = f"http://{args.host}:{args.port}/api/relative_nav"

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
