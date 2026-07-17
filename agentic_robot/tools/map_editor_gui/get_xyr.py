#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Grid Map 2D Pose 获取工具 功能：从 grid_map 图像交互式获取 2D 位姿 (x, y, yaw)"""

import os
import re
import sys
import yaml
import math
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QAction, QToolBar, QLabel,
    QFileDialog, QMessageBox, QStatusBar, QGraphicsScene, QGraphicsView,
    QGraphicsLineItem, QGraphicsEllipseItem, QGraphicsPolygonItem
)
from PyQt5.QtGui import QPixmap, QImage, QPen, QPolygonF, QTransform
from PyQt5.QtCore import Qt, QPointF, pyqtSignal, QTimer
from PIL import Image


class GraphicsView(QGraphicsView):
    """支持鼠标点击交互的 GraphicsView."""

    click_signal = pyqtSignal(QPointF)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.interactive_mode = False
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        # 启用滚轮缩放
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setMinimumSize(100, 100)

    def enable_interactive_mode(self, enable=True):
        """启用/禁用交互模式."""
        self.interactive_mode = enable
        if enable:
            self.setDragMode(QGraphicsView.NoDrag)
            self.setCursor(Qt.CrossCursor)
        else:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.unsetCursor()

    def wheelEvent(self, event):
        """鼠标滚轮缩放."""
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1.0 / zoom_factor, 1.0 / zoom_factor)

    def mousePressEvent(self, event):
        if self.interactive_mode and event.button() == Qt.LeftButton:
            pos = self.mapToScene(event.pos())
            self.click_signal.emit(pos)
        else:
            super().mousePressEvent(event)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.map_loaded = False
        self.resolution = None
        self.origin = None
        self.map_width = None
        self.map_height = None
        self.click_points = []  # 存储点击的点
        self.output_file = "pose2d_output.txt"

        # 坐标转换参数（根据 grid_map_gen.py 的逻辑）
        # PGM y轴翻转: py_map = height - 1 - py_pixel
        # x = origin_x + px * resolution
        # y = origin_y + (height - 1 - py) * resolution

        self.init_ui()
        self.auto_load_default_map()

    def init_ui(self):
        """初始化UI."""
        self.setWindowTitle("Grid Map 2D Pose 获取工具")
        self.setGeometry(100, 100, 1200, 800)

        # 创建菜单栏
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu("文件")

        self.import_pgm_action = QAction("导入 PGM/YAML", self)
        self.import_pgm_action.setShortcut("Ctrl+O")
        self.import_pgm_action.triggered.connect(self.import_map)
        file_menu.addAction(self.import_pgm_action)

        exit_action = QAction("退出", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # 创建工具栏
        toolbar = QToolBar("工具栏")
        self.addToolBar(toolbar)

        self.get_pose_action = QAction("Get Pose2D", self)
        self.get_pose_action.setCheckable(True)
        self.get_pose_action.triggered.connect(self.toggle_pose_mode)
        toolbar.addAction(self.get_pose_action)

        toolbar.addSeparator()

        clear_action = QAction("清除标记", self)
        clear_action.triggered.connect(self.clear_markers)
        toolbar.addAction(clear_action)

        # 创建状态栏
        self.status_label = QLabel("请导入地图或加载默认地图")
        self.statusBar().addWidget(self.status_label)

        # 创建图形视图
        self.graphics_view = GraphicsView(self)
        self.setCentralWidget(self.graphics_view)
        self.graphics_scene = QGraphicsScene(self)
        self.graphics_view.setScene(self.graphics_scene)
        self.graphics_view.click_signal.connect(self.on_map_click)

        # 标记项列表
        self.marker_items = []
        self.line_items = []

    def auto_load_default_map(self):
        """自动加载同目录下的默认地图."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_pgm = os.path.join(script_dir, "grid_map.pgm")
        default_yaml = os.path.join(script_dir, "grid_map.yaml")

        if os.path.exists(default_pgm) and os.path.exists(default_yaml):
            print(f"[INFO] 自动加载默认地图: {default_pgm}")
            self.load_map(default_pgm, default_yaml)
        else:
            print("[INFO] 未找到默认地图，请手动导入")

    def import_map(self):
        """导入 PGM/YAML 文件."""
        script_dir = os.path.dirname(os.path.abspath(__file__))

        pgm_file, _ = QFileDialog.getOpenFileName(
            self, "选择 PGM 文件", script_dir,
            "PGM Files (*.pgm);;All Files (*)"
        )

        if not pgm_file:
            return

        # 查找同名的 yaml 文件
        yaml_file = pgm_file.replace('.pgm', '.yaml')
        if not os.path.exists(yaml_file):
            yaml_file, _ = QFileDialog.getOpenFileName(
                self, "选择 YAML 文件", os.path.dirname(pgm_file),
                "YAML Files (*.yaml *.yml);;All Files (*)"
            )
            if not yaml_file:
                return

        self.load_map(pgm_file, yaml_file)

    def load_map(self, pgm_file, yaml_file):
        """加载地图."""
        try:
            # 读取 YAML
            with open(yaml_file, 'r') as f:
                yaml_data = yaml.safe_load(f)

            self.resolution = yaml_data['resolution']
            self.origin = yaml_data['origin'][:2]  # 只取 x, y
            print(f"[INFO] 加载地图参数:")
            print(f"  - 分辨率: {self.resolution}")
            print(f"  - 原点: {self.origin}")

            # 直接用 QPixmap 加载 PGM（显示为灰度图）
            pixmap = QPixmap(pgm_file)
            if pixmap.isNull():
                raise Exception("QPixmap 加载失败")

            self.map_width = pixmap.width()
            self.map_height = pixmap.height()
            print(f"  - 尺寸: {self.map_width} x {self.map_height}")

            # 清理之前的标记
            self.marker_items.clear()
            self.line_items.clear()
            self.click_points = []

            # 显示图像
            self.graphics_scene.clear()
            self.graphics_scene.setSceneRect(
                0, 0, self.map_width, self.map_height)
            self.graphics_scene.addPixmap(pixmap)
            self.graphics_view.setSceneRect(
                0, 0, self.map_width, self.map_height)

            self.map_loaded = True
            self.load_and_draw_saved_poses()
            self.fit_map_in_view()
            QTimer.singleShot(0, self.fit_map_in_view)

            self.status_label.setText(
                f"已加载: {os.path.basename(pgm_file)} | "
                f"分辨率: {self.resolution} | "
                f"原点: ({self.origin[0]:.2f}, {self.origin[1]:.2f})"
            )

            print(f"[INFO] 地图加载成功")

        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载地图失败: {str(e)}")
            print(f"[ERROR] 加载地图失败: {e}")

    def toggle_pose_mode(self, checked):
        """切换 Get Pose2D 模式."""
        if not self.map_loaded:
            QMessageBox.warning(self, "警告", "请先加载地图!")
            self.get_pose_action.setChecked(False)
            return

        if checked:
            self.click_points = []
            self.graphics_view.enable_interactive_mode(True)
            self.status_label.setText("点击地图选择位置和朝向 (第1点=位置, 第2点=朝向)")
            print("\n" + "=" * 50)
            print("[INFO] 进入 Get Pose2D 模式")
            print("[INFO] 第1次点击: 设置位置 | 第2次点击: 设置朝向")
            print("=" * 50)
        else:
            self.graphics_view.enable_interactive_mode(False)
            self.click_points = []
            self.status_label.setText("已退出 Get Pose2D 模式")

    def pixel_to_map_coords(self, px, py):
        """
        像素坐标转换为地图坐标.

        根据 grid_map_gen.py:
        - PGM 保存时做了 y轴翻转: v = grid[height - 1 - y, x]
        - 转换公式:
          x = origin_x + px * resolution
          y = origin_y + (height - 1 - py) * resolution
        """
        map_x = self.origin[0] + px * self.resolution
        map_y = self.origin[1] + (self.map_height - 1 - py) * self.resolution
        return map_x, map_y

    def fit_map_in_view(self):
        """将地图自动缩放到当前视图大小."""
        if not self.map_loaded:
            return
        self.graphics_view.resetTransform()
        self.graphics_view.fitInView(
            self.graphics_scene.sceneRect(),
            Qt.KeepAspectRatio)

    def map_to_pixel_coords(self, map_x, map_y):
        """地图坐标转换为像素坐标."""
        px = (map_x - self.origin[0]) / self.resolution
        py = self.map_height - 1 - (map_y - self.origin[1]) / self.resolution
        return px, py

    def load_saved_poses(self):
        """从 pose2d_output.txt 加载历史位姿."""
        poses = []
        if not os.path.exists(self.output_file):
            print(f"[INFO] 未找到历史位姿文件: {self.output_file}")
            return poses

        pattern = re.compile(
            r"x\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*"
            r"y\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*"
            r"yaw\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
        )

        try:
            with open(self.output_file, 'r') as f:
                for line_no, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue

                    match = pattern.search(line)
                    if not match:
                        print(f"[WARN] 跳过无法解析的位姿行 {line_no}: {line}")
                        continue

                    x, y, yaw = map(float, match.groups())
                    poses.append((x, y, yaw))
        except Exception as e:
            print(f"[ERROR] 读取历史位姿失败: {e}")
            return []

        print(f"[INFO] 已加载历史位姿 {len(poses)} 个")
        return poses

    def draw_saved_pose(self, map_x, map_y, yaw, index=None):
        """绘制一个历史位姿点和朝向."""
        px, py = self.map_to_pixel_coords(map_x, map_y)

        marker = QGraphicsEllipseItem(-3.0, -3.0, 6.0, 6.0)
        marker.setPen(QPen(Qt.darkYellow, 1))
        marker.setBrush(Qt.yellow)
        marker.setPos(QPointF(px, py))
        self.graphics_scene.addItem(marker)
        self.marker_items.append(marker)

        arrow_len = 20.0
        end_px = px + arrow_len * math.cos(yaw)
        end_py = py - arrow_len * math.sin(yaw)
        self.draw_arrow(
            QPointF(
                px, py), QPointF(
                end_px, end_py), color=Qt.darkYellow)

        if index is not None:
            print(
                f"[INFO] 历史点位 {index}: map=({map_x:.3f}, {map_y:.3f}), "
                f"yaw={yaw:.3f} rad -> pixel=({px:.1f}, {py:.1f})"
            )

    def load_and_draw_saved_poses(self):
        """加载并绘制所有历史位姿."""
        poses = self.load_saved_poses()
        for index, (map_x, map_y, yaw) in enumerate(poses, 1):
            self.draw_saved_pose(map_x, map_y, yaw, index=index)

    def on_map_click(self, scene_pos):
        """处理地图点击."""
        if len(self.click_points) >= 2:
            # 重置
            self.click_points = []
            self.marker_items.clear()
            self.line_items.clear()

        self.click_points.append(scene_pos)

        # 判断是起点还是终点
        if len(self.click_points) == 1:
            # 起点 - 红色
            marker = QGraphicsEllipseItem(-2.5, -2.5, 5, 5)
            marker.setPen(QPen(Qt.red, 1))
            marker.setBrush(Qt.red)
            print(
                f"\n[INFO] 第1点(起点)已设置 (像素): ({scene_pos.x():.1f}, {scene_pos.y():.1f})")
        else:
            # 终点 - 绿色
            marker = QGraphicsEllipseItem(-2.5, -2.5, 5, 5)
            marker.setPen(QPen(Qt.green, 1))
            marker.setBrush(Qt.green)

        marker.setPos(scene_pos)
        self.graphics_scene.addItem(marker)
        self.marker_items.append(marker)

        if len(self.click_points) == 1:
            self.status_label.setText("第1点已设置，请点击第2点设置朝向")

        elif len(self.click_points) == 2:
            p1, p2 = self.click_points
            self.compute_and_output_pose(p1, p2)
            self.status_label.setText("位姿已计算并保存，可继续点击或关闭模式")

    def compute_and_output_pose(self, p1, p2):
        """计算并输出位姿."""
        # 像素坐标
        px1, py1 = p1.x(), p1.y()
        px2, py2 = p2.x(), p2.y()

        # 转换为地图坐标
        map_x, map_y = self.pixel_to_map_coords(px1, py1)
        map_x2, map_y2 = self.pixel_to_map_coords(px2, py2)

        # 计算 yaw (在地图坐标系中)
        dx = map_x2 - map_x
        dy = map_y2 - map_y
        yaw = math.atan2(dy, dx)

        # 按地图坐标系 yaw 重新绘制箭头，确保显示方向与保存值一致
        end_px = px1 + 20.0 * math.cos(yaw)
        end_py = py1 - 20.0 * math.sin(yaw)
        self.draw_arrow(QPointF(px1, py1), QPointF(end_px, end_py))

        # 输出到终端
        print("\n" + "=" * 50)
        print("[RESULT] 位姿计算结果 (map坐标系):")
        print("=" * 50)
        print(f"  像素坐标: ({px1:.1f}, {py1:.1f})")
        print(f"  地图坐标: x = {map_x:.6f}")
        print(f"           y = {map_y:.6f}")
        print(f"           yaw = {yaw:.6f} rad")
        print(f"           yaw = {math.degrees(yaw):.2f} deg")
        print("=" * 50)

        # 保存到文件
        self.save_pose_to_file(map_x, map_y, yaw)

        # 更新状态
        self.status_label.setText(
            f"x={map_x:.3f}, y={map_y:.3f}, yaw={yaw:.4f} rad ({math.degrees(yaw):.1f}°)"
        )

    def save_pose_to_file(self, x, y, yaw):
        """保存位姿到文件."""
        try:
            with open(self.output_file, 'a') as f:
                timestamp = self.get_timestamp()
                f.write(f"[{timestamp}] x={x:.6f}, y={y:.6f}, yaw={yaw:.6f}\n")
            print(f"[INFO] 已保存到: {self.output_file}")
        except Exception as e:
            print(f"[ERROR] 保存失败: {e}")

    def get_timestamp(self):
        """获取时间戳."""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def draw_arrow(self, p1, p2, color=Qt.blue):
        """绘制带箭头的线段."""
        # 绘制主线
        line = QGraphicsLineItem(p1.x(), p1.y(), p2.x(), p2.y())
        line.setPen(QPen(color, 2))
        self.graphics_scene.addItem(line)
        self.line_items.append(line)

        # 计算箭头
        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        length = math.sqrt(dx * dx + dy * dy)
        if length < 1e-6:
            return

        # 箭头长度和角度
        arrow_len = 15.0
        arrow_angle = math.pi / 6  # 30度

        # 箭头两翼端点
        angle = math.atan2(dy, dx)
        a1 = angle + math.pi - arrow_angle
        a2 = angle + math.pi + arrow_angle

        # 箭头起点（在线段终点往前一点）
        arrow_tip_x = p2.x()
        arrow_tip_y = p2.y()

        # 左翼
        x1 = arrow_tip_x + arrow_len * math.cos(a1)
        y1 = arrow_tip_y + arrow_len * math.sin(a1)

        # 右翼
        x2 = arrow_tip_x + arrow_len * math.cos(a2)
        y2 = arrow_tip_y + arrow_len * math.sin(a2)

        # 绘制左翼
        arrow1 = QGraphicsLineItem(arrow_tip_x, arrow_tip_y, x1, y1)
        arrow1.setPen(QPen(color, 2))
        self.graphics_scene.addItem(arrow1)
        self.line_items.append(arrow1)

        # 绘制右翼
        arrow2 = QGraphicsLineItem(arrow_tip_x, arrow_tip_y, x2, y2)
        arrow2.setPen(QPen(color, 2))
        self.graphics_scene.addItem(arrow2)
        self.line_items.append(arrow2)

    def clear_markers(self):
        """清除所有标记."""
        for item in self.marker_items:
            self.graphics_scene.removeItem(item)
        for item in self.line_items:
            self.graphics_scene.removeItem(item)
        self.marker_items.clear()
        self.line_items.clear()
        self.click_points.clear()

    def closeEvent(self, event):
        """关闭窗口."""
        if self.graphics_view.interactive_mode:
            self.graphics_view.enable_interactive_mode(False)
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
