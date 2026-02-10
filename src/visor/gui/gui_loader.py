import tkinter as tk
from enum import Enum
from datetime import datetime
import os
import signal
import subprocess
import threading
import queue
from pathlib import Path


# --- RobotState Enum with properties ---
class RobotState(Enum):
    IDLE = {'color': 'grey', 'text': 'Idle'}
    RUNNING = {'color': 'green', 'text': 'Running'}
    PAUSED = {'color': 'red', 'text': 'Paused'}

    @property
    def color(self):
        return self.value['color']

    @property
    def text(self):
        return self.value['text']


# --- Global Variables ---
state = RobotState.IDLE
led = None
status_text = None
led_canvas = None
log_text = None
root = None

nav_proc = None
_nav_reader_thread = None

log_q: "queue.Queue[str]" = queue.Queue()
ui_q: "queue.Queue[callable]" = queue.Queue()


# --- UI-safe log append (must be called from main thread only) ---
def log(message: str):
    """Append a message to the log with timestamp (main-thread only)."""
    if log_text is None:
        return
    timestamp = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
    log_entry = f"{timestamp} {message}"

    log_text.config(state="normal")
    log_text.insert(tk.END, log_entry + "\n")
    log_text.see(tk.END)
    log_text.config(state="disabled")


# --- Thread-safe helpers (can be called from any thread) ---
def log_threadsafe(message: str):
    log_q.put(message)

def ui_threadsafe(fn):
    """Schedule a callable to run on the Tk main thread."""
    ui_q.put(fn)


# --- Update LED and Status (main thread only) ---
def update_status():
    if led_canvas is None or status_text is None or led is None:
        return
    led_canvas.itemconfig(led, fill=state.color)
    status_text.config(text=state.text)


def _start_pollers():
    """Run queued UI updates + logs on the Tk main thread."""
    def _poll():
        # Run any UI callbacks
        try:
            while True:
                fn = ui_q.get_nowait()
                try:
                    fn()
                except Exception as e:
                    # Even this is safe (main thread)
                    log(f"UI callback error: {e}")
        except queue.Empty:
            pass

        # Append logs
        try:
            while True:
                msg = log_q.get_nowait()
                log(msg)
        except queue.Empty:
            pass

        root.after(50, _poll)

    root.after(50, _poll)


def _start_ros2_launch() -> bool:
    """Starts the ros2 launch process and streams its stdout to thread-safe queue."""
    global nav_proc, _nav_reader_thread

    setup_script = Path("src/visor/simulation/install/setup.bash").resolve()
    if not setup_script.exists():
        log("ERROR: setup.bash not found at:")
        log(str(setup_script))
        return False

    cmd = f'source "{setup_script}" && ros2 launch team1_sim_bringup team1_nav2.launch.py'

    try:
        nav_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,  # separate process group => easier kill
        )
    except Exception as e:
        log(f"ERROR: Failed to start ros2 launch: {e}")
        nav_proc = None
        return False

    def _reader():
        try:
            assert nav_proc is not None
            assert nav_proc.stdout is not None
            for line in nav_proc.stdout:
                log_threadsafe(line.rstrip())
        finally:
            rc = nav_proc.poll() if nav_proc else None
            log_threadsafe(f"[nav2] process exited (code={rc})")

            # When it exits, set UI state back to IDLE on main thread
            def _set_idle():
                global state, nav_proc
                nav_proc = None
                state = RobotState.IDLE
                update_status()
            ui_threadsafe(_set_idle)

    _nav_reader_thread = threading.Thread(target=_reader, daemon=True)
    _nav_reader_thread.start()
    return True


def stop_navigation():
    """Graceful stop (SIGINT) for the ros2 launch process group."""
    global nav_proc, state
    if nav_proc is None or nav_proc.poll() is not None:
        log("Navigation is not running.")
        nav_proc = None
        state = RobotState.IDLE
        update_status()
        return

    log("Stopping navigation (SIGINT)...")
    try:
        os.killpg(nav_proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    except Exception as e:
        log(f"Stop error: {e}")


# --- Button Callbacks ---
def start_navigation():
    global state

    if state == RobotState.RUNNING:
        log("Already running")
        return

    # Start process first; only set RUNNING if successful
    ok = _start_ros2_launch()
    if not ok:
        state = RobotState.IDLE
        update_status()
        return

    state = RobotState.RUNNING
    update_status()
    log("Navigation started.")


def pause_navigation():
    # NOTE: This only updates UI state. It does NOT pause ROS2 nodes.
    # If you want true pause/resume, we can implement SIGSTOP/SIGCONT,
    # but it can be risky for ROS timing / sim.
    global state
    if state == RobotState.PAUSED:
        log("Already paused")
        return
    state = RobotState.PAUSED
    update_status()
    log("Navigation paused (UI only).")


def emergency_stop():
    global nav_proc, state

    if nav_proc is None or nav_proc.poll() is not None:
        log("Emergency Stop: navigation is not running.")
        nav_proc = None
        state = RobotState.IDLE
        update_status()
        return

    log("EMERGENCY STOP: sending SIGINT to ros2 launch...")
    try:
        os.killpg(nav_proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    except Exception as e:
        log(f"Emergency stop error: {e}")

    # UI feedback popup
    top = tk.Toplevel(root)
    top.title("Emergency Stop")
    top.geometry("320x120")
    tk.Label(top, text="Robot stop signal sent.", font=("Arial", 12)).pack(pady=10)
    tk.Button(top, text="Close", width=15, command=top.destroy).pack(pady=5)


def on_close():
    # Ensure we stop ros2 when GUI closes
    stop_navigation()
    root.destroy()


# --- Main GUI ---
def main():
    global led, led_canvas, status_text, log_text, root

    root = tk.Tk()
    root.title("V.I.S.O.R. Control Station")
    root.geometry("800x430")
    root.resizable(False, False)
    root.protocol("WM_DELETE_WINDOW", on_close)

    # --- Title ---
    tk.Label(root, text="V.I.S.O.R. Control Station", font=("Arial", 18, "bold")).pack(pady=10)

    # --- Top Frame: Status + Logs ---
    top_frame = tk.Frame(root)
    top_frame.pack(padx=10, pady=10, fill="x")

    # Status Frame
    status_frame = tk.LabelFrame(top_frame, text="Status", font=("Arial", 12, "bold"), padx=20, pady=20)
    status_frame.pack(side="left", padx=10, fill="y")
    status_frame.configure(width=200, height=200)
    status_frame.pack_propagate(False)

    # LED
    led_canvas = tk.Canvas(status_frame, width=60, height=60, highlightthickness=0)
    led_canvas.pack(pady=10)
    led = led_canvas.create_oval(5, 5, 55, 55, fill=state.color)

    # Status Text
    status_text = tk.Label(status_frame, text=state.text, font=("Arial", 12))
    status_text.pack(pady=5)

    # Log Frame
    log_frame = tk.LabelFrame(top_frame, text="Logs", font=("Arial", 12, "bold"), padx=10, pady=10)
    log_frame.pack(side="left", padx=10, fill="both", expand=True)
    log_frame.configure(height=200)
    log_frame.pack_propagate(False)

    # Scrollable Text Widget
    log_text = tk.Text(log_frame, state="disabled", wrap="word")
    log_text.pack(side="left", fill="both", expand=True)
    scrollbar = tk.Scrollbar(log_frame, command=log_text.yview)
    scrollbar.pack(side="right", fill="y")
    log_text.config(yscrollcommand=scrollbar.set)

    # --- Bottom Frame: Controls ---
    button_frame = tk.LabelFrame(root, text="Controls", font=("Arial", 12, "bold"), padx=20, pady=20)
    button_frame.pack(fill="x", padx=20, pady=20)

    start_button = tk.Button(
        button_frame,
        text="Start Navigation",
        width=15,
        height=2,
        font=("Arial", 12),
        command=start_navigation
    )
    pause_button = tk.Button(
        button_frame,
        text="Pause Navigation",
        width=15,
        height=2,
        font=("Arial", 12),
        command=pause_navigation
    )
    stop_button = tk.Button(
        button_frame,
        text="Stop Navigation",
        width=15,
        height=2,
        font=("Arial", 12),
        command=stop_navigation
    )
    emergency_stop_button = tk.Button(
        button_frame,
        text="EMERGENCY STOP",
        width=20,
        height=2,
        bg="red",
        fg="white",
        font=("Arial", 14, "bold"),
        command=emergency_stop
    )

    start_button.pack(side="left", padx=10)
    pause_button.pack(side="left", padx=10)
    stop_button.pack(side="left", padx=10)
    emergency_stop_button.pack(side="right", padx=10)

    # Start pollers (queue -> UI)
    _start_pollers()

    # Initial log
    log("System initialized. Status: Idle")

    root.mainloop()


if __name__ == "__main__":
    main()
