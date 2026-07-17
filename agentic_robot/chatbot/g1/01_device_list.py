"""
ALSA: https://zhuanlan.zhihu.com/p/813448431
"""

import argparse
from typing import List, Optional

from audio.misc import list_audio_devices


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=str, default="USB")
    args = parser.parse_args(argv)

    print(f"正在扫描音频设备: {args.device} ...")
    device_list = list_audio_devices(args.device)

    for dev in device_list:
        print(dev)


if __name__ == "__main__":
    main()
