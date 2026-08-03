"""Integration test tổng hợp tiếng Việt bằng Piper."""

import pytest

try:
    import numpy
    import piper  # noqa: F401
    import sounddevice  # noqa: F401
except (ImportError, OSError) as error:
    pytest.skip(
        f"Thiếu môi trường voice_llm_env: {error}",
        allow_module_level=True,
    )

from voice_llm_agent.infrastructure.tts.piper_adapter import PiperTTS


def test_synthesize_returns_non_empty_audio() -> None:
    tts = PiperTTS()
    chunks = list(tts._voice.synthesize("Xin chào, tôi là robot."))

    assert chunks
    audio = numpy.concatenate(
        [chunk.audio_float_array for chunk in chunks]
    )
    assert audio.size > 0
