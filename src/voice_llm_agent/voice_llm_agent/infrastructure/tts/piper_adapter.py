"""Adapter tổng hợp tiếng Việt bằng Piper."""

import os

import numpy
import sounddevice
from piper import PiperVoice

from ...domain.ports import TTSPort


class PiperTTS(TTSPort):
    """Tổng hợp và phát tiếng nói với một voice được tải một lần."""

    def __init__(
        self,
        voice_path: str = (
            "~/.local/share/piper-voices/vi_VN-vais1000-medium.onnx"
        ),
    ) -> None:
        self.voice_path = os.path.expanduser(voice_path)
        self._voice = PiperVoice.load(self.voice_path)

    def speak(self, text: str) -> None:
        chunks = list(self._voice.synthesize(text))
        if not chunks:
            return

        audio = numpy.concatenate(
            [chunk.audio_float_array for chunk in chunks]
        )
        sounddevice.play(
            audio,
            samplerate=self._voice.config.sample_rate,
        )
        sounddevice.wait()
