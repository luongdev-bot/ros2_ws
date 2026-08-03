"""Adapter nhận dạng tiếng Việt bằng faster-whisper."""

import numpy
import sounddevice
from faster_whisper import WhisperModel

from ...domain.ports import ASRPort


class WhisperASR(ASRPort):
    """Ghi âm từ microphone và nhận dạng bằng một model được tái sử dụng."""

    def __init__(
        self,
        model_size: str = "small",
        sample_rate: int = 16000,
        record_seconds: float = 5.0,
        language: str = "vi",
        device: str = "cpu",
        compute_type: str = "int8",
        silence_rms_threshold: float = 0.05,
        initial_prompt: str = (
            "Đây là một đoạn hội thoại tiếng Việt với robot."
        ),
    ) -> None:
        self.sample_rate = sample_rate
        self.record_seconds = record_seconds
        self.language = language
        self.last_rms: float = 0.0
        self._silence_rms_threshold = silence_rms_threshold
        self._initial_prompt = initial_prompt
        self._model = WhisperModel(
            model_size,
            device=device,
            compute_type=compute_type,
        )

    def _record(self) -> numpy.ndarray:
        audio = sounddevice.rec(
            int(self.record_seconds * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32",
        )
        sounddevice.wait()
        return audio.flatten()

    def _transcribe(self, audio) -> str:
        segments, _ = self._model.transcribe(
            audio,
            language=self.language,
            initial_prompt=self._initial_prompt,
            vad_filter=True,
            condition_on_previous_text=False,
        )
        return " ".join(
            segment.text.strip()
            for segment in segments
            if segment.text.strip()
        ).strip()

    def listen(self) -> str:
        audio = self._record()
        self.last_rms = float(
            numpy.sqrt(numpy.mean(numpy.square(audio)))
        )
        if self.last_rms < self._silence_rms_threshold:
            return ""
        return self._transcribe(audio)
