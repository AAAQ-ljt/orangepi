"""音频播放封装：比赛只需要播放预录制语音。"""
from __future__ import annotations

import subprocess


class AudioPlayer:
    def __init__(self, wav_path: str = "/root/dev/assets/announce.wav"):
        self.wav_path = wav_path

    def play(self) -> None:
        # 使用 aplay 播放，避免复杂音频服务
        subprocess.run(["aplay", self.wav_path], check=False)
