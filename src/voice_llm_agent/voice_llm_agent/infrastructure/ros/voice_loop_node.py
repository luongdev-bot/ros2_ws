"""ROS 2 voice loop connecting microphone ASR and reply TTS."""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..asr.whisper_adapter import WhisperASR
from ..tts.piper_adapter import PiperTTS


class VoiceLoopNode(Node):
    """Record utterances and speak replies through the text-agent topics."""

    def __init__(self) -> None:
        super().__init__("voice_loop")

        self.declare_parameter(
            "user_utterance_topic",
            "/tool_executor/user_utterance",
        )
        self.declare_parameter(
            "agent_reply_topic",
            "/tool_executor/agent_reply",
        )
        self.declare_parameter("whisper_model_size", "small")
        self.declare_parameter("whisper_language", "vi")
        self.declare_parameter(
            "whisper_initial_prompt",
            "Đây là một đoạn hội thoại tiếng Việt với robot.",
        )
        self.declare_parameter("whisper_device", "cpu")
        self.declare_parameter("whisper_compute_type", "int8")
        self.declare_parameter("record_seconds", 5.0)
        self.declare_parameter("silence_rms_threshold", 0.05)
        self.declare_parameter(
            "piper_voice_path",
            "~/.local/share/piper-voices/vi_VN-vais1000-medium.onnx",
        )

        user_utterance_topic = str(
            self.get_parameter("user_utterance_topic").value
        )
        agent_reply_topic = str(
            self.get_parameter("agent_reply_topic").value
        )
        self._silence_rms_threshold = float(
            self.get_parameter("silence_rms_threshold").value
        )

        self._asr = WhisperASR(
            model_size=str(
                self.get_parameter("whisper_model_size").value
            ),
            language=str(self.get_parameter("whisper_language").value),
            initial_prompt=str(
                self.get_parameter("whisper_initial_prompt").value
            ),
            device=str(self.get_parameter("whisper_device").value),
            compute_type=str(
                self.get_parameter("whisper_compute_type").value
            ),
            record_seconds=float(
                self.get_parameter("record_seconds").value
            ),
            silence_rms_threshold=self._silence_rms_threshold,
        )
        self._tts = PiperTTS(
            voice_path=str(self.get_parameter("piper_voice_path").value)
        )

        self._utterance_publisher = self.create_publisher(
            String,
            user_utterance_topic,
            10,
        )
        self._agent_reply_subscription = self.create_subscription(
            String,
            agent_reply_topic,
            self._on_agent_reply,
            10,
        )

        self._audio_device_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._listen_thread = threading.Thread(
            target=self._listen_loop,
            daemon=True,
        )
        self._listen_thread.start()

    def _on_agent_reply(self, message: String) -> None:
        worker = threading.Thread(
            target=self._speak_reply,
            args=(message.data,),
            daemon=True,
            name="voice_loop_tts",
        )
        worker.start()

    def _speak_reply(self, reply: str) -> None:
        try:
            with self._audio_device_lock:
                self._tts.speak(reply)
        except Exception as error:
            self.get_logger().error(
                f"Không thể phát phản hồi bằng giọng nói: {error}"
            )

    def _listen_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                with self._audio_device_lock:
                    text = self._asr.listen().strip()
                rms = self._asr.last_rms
                self.get_logger().info(
                    f"RMS đo được: {rms:.4f} "
                    f"(ngưỡng im lặng: {self._silence_rms_threshold})"
                )
                if not text:
                    continue
                self._utterance_publisher.publish(String(data=text))
                self.get_logger().info(f"Đã nhận dạng phát ngôn: {text}")
            except Exception as error:
                self.get_logger().error(
                    f"Không thể nhận dạng giọng nói: {error}"
                )
                time.sleep(1.0)

    def destroy_node(self) -> None:
        self._stop_event.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = VoiceLoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
