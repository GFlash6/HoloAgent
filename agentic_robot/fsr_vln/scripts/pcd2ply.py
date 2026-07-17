import open3d as o3d
import argparse
import os
import sys


def convert_pcd_to_ply(input_path, output_path):
    """
    读取 .pcd 点云文件并将其另存为 .ply 文件。

    Args:
        input_path (str): 输入的 .pcd 文件路径。
        output_path (str): 输出的 .ply 文件路径。
    """
    # 检查输入文件是否存在
    if not os.path.exists(input_path):
        print(f"错误: 输入文件不存在 '{input_path}'", file=sys.stderr)
        return

    print(f"正在读取 PCD 文件: {input_path}")
    try:
        # 1. 使用 open3d 读取 PCD 文件
        pcd = o3d.io.read_point_cloud(input_path)
    except Exception as e:
        print(f"读取文件时出错: {e}", file=sys.stderr)
        return

    if not pcd.has_points():
        print(f"警告: PCD 文件 '{input_path}' 为空或无法解析。", file=sys.stderr)
        return

    print(f"正在写入 PLY 文件: {output_path}")
    try:
        # 2. 使用 open3d 写入 PLY 文件
        # write_ascii=True 可以确保更好的兼容性和可读性
        o3d.io.write_point_cloud(output_path, pcd, write_ascii=True)
        print(f"转换成功！文件已保存至: {output_path}")
    except Exception as e:
        print(f"写入文件时出错: {e}", file=sys.stderr)


if __name__ == "__main__":
    # --- 设置命令行参数解析 ---
    parser = argparse.ArgumentParser(
        description="将 PCD 点云文件转换为 PLY 格式。",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "input",
        type=str,
        help="输入的 .pcd 文件路径。"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        help="输出的 .ply 文件路径。\n如果未提供，将使用与输入文件相同的名称（扩展名除外）。"
    )

    args = parser.parse_args()

    input_file = args.input

    # --- 确定输出文件路径 ---
    if args.output:
        output_file = args.output
    else:
        # 如果未指定输出路径，则根据输入路径自动生成
        # 例如：/path/to/cloud.pcd -> /path/to/cloud.ply
        base_name = os.path.splitext(input_file)[0]
        output_file = base_name + ".ply"

    # --- 执行转换 ---
    convert_pcd_to_ply(input_file, output_file)
