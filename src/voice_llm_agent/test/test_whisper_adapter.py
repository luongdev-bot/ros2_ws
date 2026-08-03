"""Integration test Piper -> WAV -> faster-whisper."""

import wave
from unittest.mock import patch

import pytest

try:
    import numpy
    import piper  # noqa: F401
    import sounddevice  # noqa: F401
    import faster_whisper  # noqa: F401
except (ImportError, OSError) as error:
    pytest.skip(
        f"Thiếu môi trường voice_llm_env: {error}",
        allow_module_level=True,
    )

from voice_llm_agent.infrastructure.asr.whisper_adapter import WhisperASR
from voice_llm_agent.infrastructure.tts.piper_adapter import PiperTTS


def test_listen_returns_empty_string_for_silence() -> None:
    silence = numpy.zeros(16000 * 3, dtype=numpy.float32)

    with patch(
        "voice_llm_agent.infrastructure.asr.whisper_adapter.WhisperModel"
    ) as whisper_model:
        asr = WhisperASR()
        with patch.object(asr, "_record", return_value=silence):
            assert asr.listen() == ""

    assert asr.last_rms == pytest.approx(0.0, abs=1e-12)
    whisper_model.return_value.transcribe.assert_not_called()


def test_transcribes_known_piper_speech(tmp_path) -> None:
    tts = PiperTTS()
    chunks = list(tts._voice.synthesize("Xin chào robot."))
    assert chunks

    audio = numpy.concatenate(
        [chunk.audio_float_array for chunk in chunks]
    )
    expected_rms = float(numpy.sqrt(numpy.mean(numpy.square(audio))))
    pcm = numpy.clip(
        audio * 32767.0,
        -32768,
        32767,
    ).astype("<i2")
    audio_path = tmp_path / "xin_chao_robot.wav"
    with wave.open(str(audio_path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(tts._voice.config.sample_rate)
        wav_file.writeframes(pcm.tobytes())

    asr = WhisperASR()
    transcribe_file = asr._transcribe
    with (
        patch.object(asr, "_record", return_value=audio),
        patch.object(
            asr,
            "_transcribe",
            side_effect=lambda _: transcribe_file(str(audio_path)),
        ),
    ):
        transcript = asr.listen().lower()

    assert asr.last_rms == pytest.approx(expected_rms)
    assert "chào" in transcript or "robot" in transcript
