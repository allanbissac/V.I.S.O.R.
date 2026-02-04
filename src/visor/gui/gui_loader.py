import tkinter as tk
from enum import Enum
from datetime import datetime


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


# --- Helper to append log ---
def log(message):
    """Append a message to the log with timestamp."""
    timestamp = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
    log_entry = f"{timestamp} {message}"
    
    log_text.config(state="normal")
    log_text.insert(tk.END, log_entry + "\n")
    log_text.see(tk.END)
    log_text.config(state="disabled")


# --- Update LED and Status ---
def update_status():
    led_canvas.itemconfig(led, fill=state.color)
    status_text.config(text=state.text)


# --- Button Callbacks ---
def start_navigation():
    global state
    if state == RobotState.RUNNING:
        log("Already running")
        return
    state = RobotState.RUNNING
    update_status()
    log("Navigation started.")


def pause_navigation():
    global state
    if state == RobotState.PAUSED:
        log("Already paused")
        return
    state = RobotState.PAUSED
    update_status()
    log("Navigation paused.")


def emergency_stop(root):
    global state
    state = RobotState.IDLE
    update_status()
    log("Emergency Stop triggered!")

    top = tk.Toplevel(root)
    top.title("Emergency Stop")
    top.geometry("300x100")
    tk.Label(top, text="Robot has been successfully stopped.", font=("Arial", 12)).pack(pady=10)
    tk.Button(top, text="Close", width=15, command=top.destroy).pack(pady=5)


# --- Main GUI ---
def main():
    global led, led_canvas, status_text, log_text

    root = tk.Tk()
    root.title("V.I.S.O.R. Control Station")
    root.geometry("800x430")
    root.resizable(False, False)

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

    # --- Bottom Frame: Controls (spans full width) ---
    button_frame = tk.LabelFrame(root, text="Controls", font=("Arial", 12, "bold"), padx=20, pady=20)
    button_frame.pack(fill="x", padx=20, pady=20)

    # Buttons in bottom frame
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
    emergency_stop_button = tk.Button(
        button_frame,
        text="EMERGENCY STOP",
        width=20,
        height=2,
        bg="red",
        fg="white",
        font=("Arial", 14, "bold"),
        command=lambda: emergency_stop(root)
    )

    # Arrange buttons horizontally with spacing
    start_button.pack(side="left", padx=10)
    pause_button.pack(side="left", padx=10)
    emergency_stop_button.pack(side="right", padx=10)

    # Initial log
    log("System initialized. Status: Idle")

    root.mainloop()


if __name__ == "__main__":
    main()
