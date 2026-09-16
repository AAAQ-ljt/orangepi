"""音频播放：使用 ffplay 播放预录制语音（**非阻塞**）。

为什么不用 `subprocess.run`：它会阻塞调用线程直到音频播完。
控制进程在主循环里触发播报，阻塞意味着这十几秒内控制环停摆、UDP 不再处理、
failsafe 也不会触发（见 AGENTS.md §6 已知陷阱表）。
"""
from __future__ import annotations

import subprocess
from typing import Optional


class AudioPlayer:
    def __init__(self, audio_path: str = "/root/dev/voice/aa.mp3", command: str = "ffplay"):
        self.audio_path = audio_path
        self.command = command
        self._proc: Optional[subprocess.Popen] = None

    def is_playing(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def play(self) -> bool:
        """后台开始播放；已在播放时不重复启动。返回是否成功启动。"""
        if self.is_playing():
            return False
        try:
            self._proc = subprocess.Popen(
                [self.command, "-nodisp", "-autoexit", "-loglevel", "quiet", self.audio_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return True
        except FileNotFoundError:
            print(f"[AUDIO] 未找到播放器 {self.command}，无法播放 {self.audio_path}")
            self._proc = None
            return False

    def stop(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
        self._proc = None
