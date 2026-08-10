"""ROS 2 text chat adapter with Piper reply speech."""

import queue
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..tts.piper_adapter import PiperTTS


class TextChatNode(Node):
    """Send typed commands and display and speak agent replies."""

    def __init__(self) -> None:
        super().__init__("text_chat")

        self.declare_parameter(
            "user_utterance_topic",
            "/tool_executor/user_utterance",
        )
        self.declare_parameter(
            "agent_reply_topic",
            "/tool_executor/agent_reply",
        )
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
        self._closed_event = threading.Event()
        self._spin_thread = None
        self._drain_after_id = None
        self.results = queue.Queue()

        self._root = tk.Tk()
        self._root.title("Chat điều khiển JetRover")
        self._root.geometry("720x520")
        self._root.columnconfigure(0, weight=1)
        self._root.rowconfigure(0, weight=1)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        chat_frame = tk.Frame(self._root)
        chat_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        chat_frame.columnconfigure(0, weight=1)
        chat_frame.rowconfigure(0, weight=1)

        self._chat_text = tk.Text(
            chat_frame,
            state=tk.DISABLED,
            wrap=tk.WORD,
        )
        self._chat_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = tk.Scrollbar(
            chat_frame,
            orient=tk.VERTICAL,
            command=self._chat_text.yview,
        )
        scrollbar.grid(row=0, column=1, sticky="ns")
        self._chat_text.configure(yscrollcommand=scrollbar.set)

        input_frame = tk.Frame(self._root)
        input_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 10))
        input_frame.columnconfigure(0, weight=1)

        self._entry = tk.Entry(input_frame)
        self._entry.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self._entry.bind("<Return>", self._on_send)
        tk.Button(
            input_frame,
            text="Gửi",
            command=self._on_send,
        ).grid(row=0, column=1)
        self._entry.focus_set()

        self._drain_after_id = self.after(150, self._drain_replies)

    def _append_chat_line(self, line: str) -> None:
        self._chat_text.configure(state=tk.NORMAL)
        self._chat_text.insert(tk.END, line + "\n")
        self._chat_text.see(tk.END)
        self._chat_text.configure(state=tk.DISABLED)

    def _on_send(self, _event=None):
        text = self._entry.get().strip()
        if not text:
            return "break"

        self._append_chat_line(f"Bạn: {text}")
        self._entry.delete(0, tk.END)
        self._utterance_publisher.publish(String(data=text))
        self.get_logger().info(f"Đã gửi lệnh: {text}")
        return "break"

    def _on_agent_reply(self, message: String) -> None:
        self.results.put(message.data)

    def after(self, delay_ms: int, callback):
        return self._root.after(delay_ms, callback)

    def _cancel_drain_callback(self) -> None:
        if self._drain_after_id is None:
            return
        try:
            self._root.after_cancel(self._drain_after_id)
        except tk.TclError:
            pass
        self._drain_after_id = None

    def _drain_replies(self) -> None:
        self._cancel_drain_callback()
        try:
            while True:
                reply = self.results.get_nowait()
                self._append_chat_line(f"Robot: {reply}")
                worker = threading.Thread(
                    target=self._speak_reply,
                    args=(reply,),
                    daemon=True,
                    name="text_chat_tts",
                )
                worker.start()
        except queue.Empty:
            pass

        if not self._stop_event.is_set():
            self._drain_after_id = self.after(
                150,
                self._drain_replies,
            )

    def _speak_reply(self, reply: str) -> None:
        try:
            with self._audio_device_lock:
                self._tts.speak(reply)
        except Exception as error:
            self.get_logger().error(
                f"Không thể phát phản hồi bằng giọng nói: {error}"
            )

    def _spin_ros(self) -> None:
        try:
            rclpy.spin(self)
        except Exception as error:
            if not self._stop_event.is_set():
                self.get_logger().error(
                    f"Luồng ROS kết thúc bất thường: {error}"
                )

    def run_gui(self) -> None:
        self._spin_thread = threading.Thread(
            target=self._spin_ros,
            daemon=True,
            name="text_chat_ros_spin",
        )
        self._spin_thread.start()
        try:
            self._root.mainloop()
        finally:
            self._on_close()

    def _on_close(self) -> None:
        if self._closed_event.is_set():
            return
        self._closed_event.set()
        self._stop_event.set()
        if rclpy.ok():
            rclpy.shutdown()
        self.destroy_node()
        if (
            self._spin_thread is not None
            and self._spin_thread.is_alive()
            and threading.current_thread() is not self._spin_thread
        ):
            self._spin_thread.join()
        self._cancel_drain_callback()
        if self._root.winfo_exists():
            self._root.destroy()

    def destroy_node(self) -> None:
        self._stop_event.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = TextChatNode()
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._on_close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
