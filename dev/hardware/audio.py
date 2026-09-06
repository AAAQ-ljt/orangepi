"""音频播放：使用 ffplay 播放预录制语音。"""
from __future__ import annotations

import subprocess


class AudioPlayer:
    def __init__(self, audio_path: str = "/root/dev/voice/aa.mp3"):
        self.audio_path = audio_path

    def play(self) -> None:
        subprocess.run(
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", self.audio_path],
            check=False,
        )
