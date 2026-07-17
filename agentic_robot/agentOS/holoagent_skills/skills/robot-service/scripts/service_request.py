#!/usr/bin/env python3
"""Helper script for robot service HTTP requests."""

from __future__ import annotations

import argparse
import json
from urllib import request


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send robot service HTTP requests")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Robot service host")
    parser.add_argument("--port", type=int, default=8000,
                        help="Robot service port")
    parser.add_argument(
        "--endpoint",
        required=True,
        help="Endpoint path, for example /api/relative_nav or /api/arm/high_wave",
    )
    parser.add_argument(
        "--method",
        default="POST",
        choices=["GET", "POST"],
        help="HTTP method",
    )
    parser.add_argument(
        "--payload",
        default="{}",
        help='JSON payload string, for example \'{"direction":"forward","distance":0.5}\'',
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Print request without sending")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    url = f"http://{args.host}:{args.port}{args.endpoint}"
    payload = json.loads(args.payload)

    print(f"URL: {url}")
    print(f"Method: {args.method}")
    print("Payload:")
    print(json.dumps(payload, ensure_ascii=False, indent=2))

    if args.dry_run:
        return 0

    data = json.dumps(payload).encode("utf-8")
    req = request.Request(
        url=url,
        data=data if args.method == "POST" else None,
        headers={"Content-Type": "application/json"},
        method=args.method,
    )

    with request.urlopen(req) as resp:
        body = resp.read().decode("utf-8", errors="replace")
        print("\nResponse:")
        print(body)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
