"""PyQt5 action-group editor — the Hiwonder layout, driving Gazebo.

Left  : one row per servo (slider for revolute joints, Open/Close for the
        gripper), each clamped to the joint's configured limits.
Right : the action table (Index / Time / ID:n ...) plus the editing buttons.

Every edit goes through :class:`~arm_motion.application.edit_session.EditSession`,
so the UI cannot produce a pose the robot profile forbids.
"""

import sys
from pathlib import Path
from typing import Dict, List, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QInputDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..application.edit_session import EditSession
from ..domain.errors import ArmMotionError, MotionAlreadyExistsError
from ..domain.joint_spec import GripperCommand, JointKind, JointSpec
from .ros_bridge import EditorRosBridge, Worker

# Slider drags are coalesced: we only command the arm once the user pauses.
LIVE_DEBOUNCE_MS = 120


class EditorWindow(QMainWindow):
    """Main window."""

    # Emitted from worker threads; Qt queues them onto the GUI thread.
    status_message = pyqtSignal(str)
    play_progress = pyqtSignal(int, int)
    play_finished = pyqtSignal()

    def __init__(self, bridge: EditorRosBridge):
        super().__init__()
        self._bridge = bridge
        self._session = EditSession(profile=bridge.profile)
        self._sliders: Dict[str, QSlider] = {}
        self._value_labels: Dict[str, QLabel] = {}
        self._updating_ui = False
        self._playing = False
        self._stop_requested = False

        self.setWindowTitle("Arm Action Group Editor (Gazebo)")
        self.resize(1180, 640)

        self._live_timer = QTimer(self)
        self._live_timer.setSingleShot(True)
        self._live_timer.setInterval(LIVE_DEBOUNCE_MS)
        self._live_timer.timeout.connect(self._send_live_pose)

        self._build_ui()
        self._connect_signals()
        self._refresh_all()

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addWidget(self._build_joint_panel(), 4)
        layout.addWidget(self._build_action_panel(), 6)
        self.setCentralWidget(central)
        self.statusBar().showMessage(
            f"Library: {self._bridge.library_dir()}"
        )

    def _build_joint_panel(self) -> QWidget:
        box = QGroupBox("Joints")
        grid = QGridLayout(box)
        grid.setColumnStretch(2, 1)

        for row, spec in enumerate(self._bridge.profile.joints):
            scale = self._bridge.profile.scale(spec.name)
            grid.addWidget(QLabel(f"<b>ID:{scale.servo_id}</b>"), row, 0)
            grid.addWidget(QLabel(spec.name), row, 1)

            if spec.kind is JointKind.GRIPPER:
                grid.addWidget(self._build_gripper_controls(spec), row, 2)
            else:
                slider = QSlider(Qt.Horizontal)
                lo, hi = sorted((scale.min_pulse, scale.max_pulse))
                slider.setRange(lo, hi)
                slider.setValue(scale.to_pulse(self._session.live_pose[spec.name]))
                slider.valueChanged.connect(
                    lambda value, name=spec.name: self._on_slider(name, value)
                )
                self._sliders[spec.name] = slider
                grid.addWidget(slider, row, 2)

            label = QLabel()
            label.setMinimumWidth(150)
            self._value_labels[spec.name] = label
            grid.addWidget(label, row, 3)

        controls = QHBoxLayout()
        self._live_check = QCheckBox("Live (follow sliders in Gazebo)")
        self._live_check.setChecked(True)
        controls.addWidget(self._live_check)

        self._read_button = QPushButton("Read angle")
        self._read_button.setToolTip(
            "Load the arm's current pose from /joint_states into the sliders"
        )
        controls.addWidget(self._read_button)

        self._center_button = QPushButton("Reset to centre")
        controls.addWidget(self._center_button)

        grid.addLayout(controls, len(self._bridge.profile.joints), 0, 1, 4)
        return box

    def _build_gripper_controls(self, spec: JointSpec) -> QWidget:
        holder = QWidget()
        row = QHBoxLayout(holder)
        row.setContentsMargins(0, 0, 0, 0)

        open_button = QPushButton("Open")
        close_button = QPushButton("Close")
        open_button.clicked.connect(
            lambda _checked, name=spec.name: self._set_gripper(
                name, GripperCommand.OPEN
            )
        )
        close_button.clicked.connect(
            lambda _checked, name=spec.name: self._set_gripper(
                name, GripperCommand.CLOSE
            )
        )
        row.addWidget(open_button)
        row.addWidget(close_button)
        row.addStretch(1)
        # No slider: this joint's only capability is open/close.
        return holder

    def _build_action_panel(self) -> QWidget:
        box = QGroupBox("Action group")
        layout = QVBoxLayout(box)

        self._table = QTableWidget(0, len(self._session.column_headers()))
        self._table.setHorizontalHeaderLabels(self._session.column_headers())
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self._table, 1)

        duration_row = QHBoxLayout()
        duration_row.addWidget(QLabel("Duration"))
        self._duration_spin = QSpinBox()
        self._duration_spin.setRange(20, 600000)
        self._duration_spin.setSingleStep(100)
        self._duration_spin.setValue(self._session.step_duration_ms)
        self._duration_spin.setSuffix(" ms")
        duration_row.addWidget(self._duration_spin)
        self._total_label = QLabel("Total: 0.0 s")
        duration_row.addWidget(self._total_label)
        duration_row.addStretch(1)
        layout.addLayout(duration_row)

        edit_grid = QGridLayout()
        self._add_button = QPushButton("Add action")
        self._update_button = QPushButton("Update action")
        self._insert_button = QPushButton("Insert action")
        self._delete_button = QPushButton("Delete action")
        self._up_button = QPushButton("Action upward")
        self._down_button = QPushButton("Action down")
        self._delete_all_button = QPushButton("Delete all")
        for index, button in enumerate(
            (
                self._add_button,
                self._update_button,
                self._insert_button,
                self._delete_button,
                self._up_button,
                self._down_button,
                self._delete_all_button,
            )
        ):
            edit_grid.addWidget(button, index // 3, index % 3)
        layout.addLayout(edit_grid)

        file_row = QHBoxLayout()
        self._motion_combo = QComboBox()
        self._motion_combo.setMinimumWidth(160)
        file_row.addWidget(QLabel("Action group"))
        file_row.addWidget(self._motion_combo, 1)
        self._open_button = QPushButton("Open action file")
        self._save_button = QPushButton("Save action file")
        self._refresh_button = QPushButton("Refresh")
        self._new_button = QPushButton("New")
        for button in (
            self._open_button,
            self._save_button,
            self._refresh_button,
            self._new_button,
        ):
            file_row.addWidget(button)
        layout.addLayout(file_row)

        run_row = QHBoxLayout()
        self._run_button = QPushButton("Run action")
        self._stop_button = QPushButton("Stop")
        self._stop_button.setEnabled(False)
        self._loop_check = QCheckBox("Loop")
        run_row.addWidget(self._run_button)
        run_row.addWidget(self._stop_button)
        run_row.addWidget(self._loop_check)
        run_row.addStretch(1)
        self._quit_button = QPushButton("Quit")
        run_row.addWidget(self._quit_button)
        layout.addLayout(run_row)

        return box

    def _connect_signals(self) -> None:
        self._table.itemSelectionChanged.connect(self._on_selection_changed)
        self._duration_spin.valueChanged.connect(self._on_duration_changed)

        self._add_button.clicked.connect(self._on_add)
        self._update_button.clicked.connect(self._on_update)
        self._insert_button.clicked.connect(self._on_insert)
        self._delete_button.clicked.connect(self._on_delete)
        self._up_button.clicked.connect(lambda: self._on_move(-1))
        self._down_button.clicked.connect(lambda: self._on_move(1))
        self._delete_all_button.clicked.connect(self._on_delete_all)

        self._open_button.clicked.connect(self._on_open)
        self._save_button.clicked.connect(self._on_save)
        self._refresh_button.clicked.connect(self._refresh_library)
        self._new_button.clicked.connect(self._on_new)

        self._run_button.clicked.connect(self._on_run)
        self._stop_button.clicked.connect(self._on_stop)
        self._quit_button.clicked.connect(self.close)

        self._read_button.clicked.connect(self._on_read_angle)
        self._center_button.clicked.connect(self._on_center)

        self.status_message.connect(self._show_status)
        self.play_progress.connect(self._on_play_progress)
        self.play_finished.connect(self._on_play_finished)

    # ------------------------------------------------------------------
    # Joint editing
    # ------------------------------------------------------------------
    def _on_slider(self, joint_name: str, value: int) -> None:
        if self._updating_ui:
            return
        applied = self._session.set_pulse(joint_name, value)
        if applied != value:
            # The joint limit is tighter than the slider range — snap back.
            slider = self._sliders[joint_name]
            self._updating_ui = True
            slider.setValue(applied)
            self._updating_ui = False
        self._refresh_value_labels()
        self._schedule_live_move()

    def _set_gripper(self, joint_name: str, command: GripperCommand) -> None:
        try:
            self._session.set_gripper(joint_name, command)
        except ArmMotionError as exc:
            self._show_status(str(exc))
            return
        self._refresh_value_labels()
        self._schedule_live_move()

    def _on_read_angle(self) -> None:
        pose = self._bridge.current_pose()
        if pose is None:
            self._show_status(
                "No /joint_states yet — is Gazebo running with the controllers?"
            )
            return
        self._session.adopt_pose(pose)
        self._refresh_joint_widgets()
        self._show_status("Sliders updated from the arm's current pose")

    def _on_center(self) -> None:
        self._session.live_pose = self._bridge.profile.home_pose()
        self._refresh_joint_widgets()
        self._schedule_live_move()

    def _schedule_live_move(self) -> None:
        if self._live_check.isChecked() and not self._playing:
            self._live_timer.start()

    def _send_live_pose(self) -> None:
        if self._playing:
            return
        pose = self._session.live_pose
        self._run_in_worker(lambda: self._bridge.goto_pose(pose))

    # ------------------------------------------------------------------
    # Step editing
    # ------------------------------------------------------------------
    def _on_duration_changed(self, value: int) -> None:
        if self._updating_ui:
            return
        self._session.set_step_duration(value)

    def _on_add(self) -> None:
        self._guarded(self._session.add_step)

    def _on_update(self) -> None:
        self._guarded(self._session.update_step)

    def _on_insert(self) -> None:
        self._guarded(self._session.insert_step)

    def _on_delete(self) -> None:
        self._guarded(self._session.delete_step)

    def _on_move(self, offset: int) -> None:
        self._guarded(lambda: self._session.move_step(offset))

    def _on_delete_all(self) -> None:
        if not len(self._session.motion):
            return
        confirm = QMessageBox.question(
            self,
            "Delete all",
            "Remove every step from this action group?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm == QMessageBox.Yes:
            self._guarded(self._session.delete_all)

    def _guarded(self, operation) -> None:
        try:
            operation()
        except ArmMotionError as exc:
            self._show_status(str(exc))
            return
        except IndexError as exc:
            self._show_status(str(exc))
            return
        self._refresh_table()

    def _on_selection_changed(self) -> None:
        if self._updating_ui:
            return
        rows = self._table.selectionModel().selectedRows()
        if not rows:
            return
        self._session.select(rows[0].row())
        self._refresh_joint_widgets()
        self._updating_ui = True
        self._duration_spin.setValue(self._session.step_duration_ms)
        self._updating_ui = False

    # ------------------------------------------------------------------
    # File operations
    # ------------------------------------------------------------------
    def _on_new(self) -> None:
        if not self._confirm_discard():
            return
        self._session.new()
        self._refresh_all()

    def _on_open(self) -> None:
        if not self._confirm_discard():
            return
        name = self._motion_combo.currentText().strip()
        if not name:
            self._show_status("No action group selected")
            return
        try:
            motion = self._bridge.load_motion.execute(name)
        except ArmMotionError as exc:
            self._show_status(f"Open failed: {exc}")
            return
        self._session.open(motion)
        self._refresh_all()
        self._show_status(f"Opened '{motion.name}' ({len(motion)} steps)")

    def _on_save(self) -> None:
        if not len(self._session.motion):
            self._show_status("Nothing to save — the action group has no steps")
            return

        suggested = self._session.motion.name
        name, accepted = QInputDialog.getText(
            self, "Save action file", "Action group name (no spaces):", text=suggested
        )
        if not accepted:
            return
        name = name.strip()
        if name.endswith(".d6a"):
            name = name[: -len(".d6a")]

        try:
            self._session.rename(name)
        except ArmMotionError as exc:
            self._show_status(str(exc))
            return

        try:
            stored = self._bridge.save_motion.execute(
                self._session.motion, overwrite=False
            )
        except MotionAlreadyExistsError:
            confirm = QMessageBox.question(
                self,
                "Overwrite",
                f"'{name}' already exists. Update it?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if confirm != QMessageBox.Yes:
                return
            try:
                stored = self._bridge.save_motion.execute(
                    self._session.motion, overwrite=True
                )
            except ArmMotionError as exc:
                self._show_status(f"Save failed: {exc}")
                return
        except ArmMotionError as exc:
            self._show_status(f"Save failed: {exc}")
            return

        self._session.mark_saved(stored)
        self._refresh_library()
        self._show_status(f"Saved {self._bridge.repository.path_for(stored.name)}")

    def _confirm_discard(self) -> bool:
        if not self._session.dirty:
            return True
        confirm = QMessageBox.question(
            self,
            "Unsaved changes",
            "The current action group has unsaved changes. Discard them?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        return confirm == QMessageBox.Yes

    # ------------------------------------------------------------------
    # Playback
    # ------------------------------------------------------------------
    def _on_run(self) -> None:
        motion = self._session.motion
        if not len(motion):
            self._show_status("Nothing to run — add a step first")
            return
        if self._playing:
            return

        self._playing = True
        self._stop_requested = False
        self._set_playing_ui(True)
        loop = self._loop_check.isChecked()

        def play() -> None:
            while True:
                self._bridge.play(
                    motion,
                    on_progress=lambda i, n: self.play_progress.emit(i, n),
                    should_cancel=lambda: self._stop_requested,
                )
                if not loop or self._stop_requested:
                    break

        self._run_in_worker(play, done=self.play_finished.emit)

    def _on_stop(self) -> None:
        self._stop_requested = True
        self._bridge.stop()
        self._show_status("Stopping...")

    def _on_play_progress(self, index: int, count: int) -> None:
        self._show_status(f"Running step {min(index + 1, count)} / {count}")

    def _on_play_finished(self) -> None:
        self._playing = False
        self._set_playing_ui(False)

    def _set_playing_ui(self, playing: bool) -> None:
        self._run_button.setEnabled(not playing)
        self._stop_button.setEnabled(playing)
        for button in (
            self._add_button,
            self._update_button,
            self._insert_button,
            self._delete_button,
            self._delete_all_button,
            self._open_button,
            self._save_button,
            self._new_button,
        ):
            button.setEnabled(not playing)

    def _run_in_worker(self, target, done=None) -> None:
        Worker(
            target=target,
            on_error=lambda message: self.status_message.emit(message),
            on_done=done or (lambda: None),
        ).start()

    # ------------------------------------------------------------------
    # Refresh
    # ------------------------------------------------------------------
    def _refresh_all(self) -> None:
        self._refresh_joint_widgets()
        self._refresh_table()
        self._refresh_library()

    def _refresh_joint_widgets(self) -> None:
        self._updating_ui = True
        try:
            pulses = self._session.live_pulses()
            for name, slider in self._sliders.items():
                if name in pulses:
                    slider.setValue(pulses[name])
        finally:
            self._updating_ui = False
        self._refresh_value_labels()

    def _refresh_value_labels(self) -> None:
        pulses = self._session.live_pulses()
        for spec in self._bridge.profile.joints:
            position = self._session.live_pose[spec.name]
            pulse = pulses.get(spec.name, 0)
            self._value_labels[spec.name].setText(
                f"{pulse:>4}  |  {spec.describe_position(position)}"
            )

    def _refresh_table(self) -> None:
        self._updating_ui = True
        try:
            rows = self._session.table_rows()
            self._table.setRowCount(len(rows))
            for row_index, row in enumerate(rows):
                for column, text in enumerate(row):
                    self._table.setItem(
                        row_index, column, QTableWidgetItem(str(text))
                    )
            if 0 <= self._session.selected_index < len(rows):
                self._table.selectRow(self._session.selected_index)
        finally:
            self._updating_ui = False

        total_ms = self._session.motion.total_duration_ms
        marker = " *" if self._session.dirty else ""
        self._total_label.setText(f"Total: {total_ms / 1000.0:.1f} s")
        self.setWindowTitle(
            f"Arm Action Group Editor (Gazebo) — {self._session.motion.name}{marker}"
        )

    def _refresh_library(self) -> None:
        current = self._motion_combo.currentText()
        self._motion_combo.blockSignals(True)
        self._motion_combo.clear()
        try:
            names = self._bridge.motion_names()
        except Exception as exc:  # noqa: BLE001
            names = []
            self._show_status(f"Could not list the library: {exc}")
        self._motion_combo.addItems(names)
        if current in names:
            self._motion_combo.setCurrentText(current)
        elif self._session.motion.name in names:
            self._motion_combo.setCurrentText(self._session.motion.name)
        self._motion_combo.blockSignals(False)
        self._refresh_table()

    def _show_status(self, message: str) -> None:
        self.statusBar().showMessage(message, 8000)

    # ------------------------------------------------------------------
    def closeEvent(self, event) -> None:  # noqa: N802 - Qt naming
        if self._playing:
            self._stop_requested = True
            self._bridge.stop()
        if not self._confirm_discard():
            event.ignore()
            return
        self._bridge.shutdown()
        event.accept()


def main(argv: Optional[List[str]] = None) -> int:
    import rclpy

    argv = list(sys.argv if argv is None else argv)
    rclpy.init(args=argv)

    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("arm_motion"))
    robot_config = Path(
        _argument(argv, "--robot-config", str(share / "config" / "jetrover_arm.yaml"))
    )
    library_dir = _argument(argv, "--library-dir", "~/ActionGroups")

    app = QApplication([a for a in argv if not a.startswith("--")])
    try:
        bridge = EditorRosBridge(robot_config, library_dir)
    except Exception as exc:  # noqa: BLE001
        QMessageBox.critical(None, "Startup failed", str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    window = EditorWindow(bridge)
    window.show()
    return app.exec_()


def _argument(argv: List[str], flag: str, default: str) -> str:
    if flag in argv:
        index = argv.index(flag)
        if index + 1 < len(argv):
            return argv[index + 1]
    prefix = f"{flag}="
    for item in argv:
        if item.startswith(prefix):
            return item[len(prefix):]
    return default


if __name__ == "__main__":
    sys.exit(main())
