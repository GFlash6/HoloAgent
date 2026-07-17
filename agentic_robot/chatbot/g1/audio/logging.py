import sys
import os
from loguru import logger

from .env import SETTINGS

WORK_DIR = os.path.expanduser(SETTINGS["work_dir"])
os.makedirs(WORK_DIR, exist_ok=True)
LOG_FILE = os.path.join(WORK_DIR, "g1.log")

__all__ = ['default_logger']


class Logger:
    """日志管理类，支持控制台和文件输出."""

    def __init__(self, level: str = "INFO", console_output: bool = True):
        """
        初始化日志器.

        Args:
            level: 日志级别，如 "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"
            console_output: 是否输出到控制台
        """
        self.level = level
        self.console_output = console_output

        # 移除默认的日志处理器
        logger.remove()

        # 配置日志格式
        log_format = (
            "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
            "<level>{level: <8}</level> | "
            "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> | "
            "<level>{message}</level>")

        # 添加控制台输出
        if self.console_output:
            logger.add(
                sys.stderr,
                format=log_format,
                level=self.level,
                colorize=True
            )

        # 添加文件日志写入（DEBUG 级别）
        logger.add(
            LOG_FILE,
            format=log_format,
            level="DEBUG",
            rotation="100 MB",
            retention="30 days",
            encoding="utf-8"
        )

        self.logger = logger

    def get_logger(self):
        """获取日志器实例."""
        return self.logger

    def debug(self, message: str, **kwargs):
        """记录 DEBUG 级别日志."""
        self.logger.opt(depth=1).debug(message, **kwargs)

    def info(self, message: str, **kwargs):
        """记录 INFO 级别日志."""
        self.logger.opt(depth=1).info(message, **kwargs)

    def warning(self, message: str, **kwargs):
        """记录 WARNING 级别日志."""
        self.logger.opt(depth=1).warning(message, **kwargs)

    def error(self, message: str, **kwargs):
        """记录 ERROR 级别日志."""
        self.logger.opt(depth=1).error(message, **kwargs)

    def critical(self, message: str, **kwargs):
        """记录 CRITICAL 级别日志."""
        self.logger.opt(depth=1).critical(message, **kwargs)

    def exception(self, message: str, **kwargs):
        """记录异常信息."""
        self.logger.opt(depth=1).exception(message, **kwargs)


# 创建默认日志器实例
default_logger = Logger()
