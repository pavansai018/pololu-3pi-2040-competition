import serial
import tkinter as tk
from tkinter import ttk
import csv
from datetime import datetime

PORT = "/dev/ttyACM0"
BAUD = 115200

BASE_SPEED_MAX = 3000

ser = serial.Serial(PORT, BAUD, timeout=0.01)

log_filename = "log_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
log_file = open(log_filename, "w", newline="")
writer = csv.writer(log_file)
writer.writerow([
    "host_time",
    "tag",
    "t_ms",
    "left_cmd",
    "right_cmd",
    "line0",
    "line1",
    "line2",
    "line3",
    "line4",
    "bumpL",
    "bumpR",
])

root = tk.Tk()
root.title("3pi+ 2040 USB Control")

speed_var = tk.DoubleVar(value=0.6)
mode_var = tk.StringVar(value="STOP")
last_log_label = tk.StringVar(value="Last LOG: (none)")
speed_label_text = tk.StringVar(value="Base speed: 0")

def send_command(cmd):
    try:
        ser.write((cmd + "\n").encode("utf-8"))
    except:
        pass

def set_mode(new_mode):
    mode_var.set(new_mode)

def update_speed_label():
    s = speed_var.get()
    base = int(s * BASE_SPEED_MAX)
    speed_label_text.set(f"Base speed: {base}")
    root.after(200, update_speed_label)

def compute_and_send():
    mode = mode_var.get()
    s = speed_var.get()
    base = int(s * BASE_SPEED_MAX)

    if mode == "STOP" or base == 0:
        cmd = "STOP"
    elif mode == "FWD":
        left = base
        right = base
        cmd = f"M {left} {right}"
    elif mode == "BACK":
        left = -base
        right = -base
        cmd = f"M {left} {right}"
    elif mode == "LEFT":
        left = -base
        right = base
        cmd = f"M {left} {right}"
    elif mode == "RIGHT":
        left = base
        right = -base
        cmd = f"M {left} {right}"
    else:
        cmd = "STOP"

    send_command(cmd)
    root.after(80, compute_and_send)

def poll_serial():
    try:
        line = ser.readline().decode(errors="ignore").strip()
    except:
        line = ""
    if line:
        parts = line.split()
        if len(parts) >= 11 and parts[0] == "LOG":
            try:
                host_time = datetime.now().isoformat()
                tag = parts[0]
                t_ms = int(parts[1])
                left_cmd = int(parts[2])
                right_cmd = int(parts[3])
                l0 = int(parts[4])
                l1 = int(parts[5])
                l2 = int(parts[6])
                l3 = int(parts[7])
                l4 = int(parts[8])
                bumpL = int(parts[9])
                bumpR = int(parts[10])
                writer.writerow([
                    host_time,
                    tag,
                    t_ms,
                    left_cmd,
                    right_cmd,
                    l0,
                    l1,
                    l2,
                    l3,
                    l4,
                    bumpL,
                    bumpR,
                ])
                last_log_label.set(
                    f"t={t_ms} L={left_cmd} R={right_cmd} "
                    f"[{l0},{l1},{l2},{l3},{l4}] "
                    f"bL={bumpL} bR={bumpR}"
                )
            except:
                pass
        else:
            print(line)
    root.after(20, poll_serial)

main_frame = ttk.Frame(root, padding=10)
main_frame.grid(row=0, column=0, sticky="nsew")

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)
for c in range(3):
    main_frame.columnconfigure(c, weight=1)

ttk.Label(main_frame, text="Speed").grid(row=0, column=0, sticky="w")
speed_slider = ttk.Scale(main_frame, from_=0.0, to=1.0,
                         variable=speed_var, orient="horizontal")
speed_slider.grid(row=0, column=1, columnspan=2, sticky="ew", padx=5)

speed_label = ttk.Label(main_frame, textvariable=speed_label_text)
speed_label.grid(row=1, column=0, columnspan=3, sticky="w")

btn_forward = ttk.Button(main_frame, text="FORWARD",
                         command=lambda: set_mode("FWD"))
btn_backward = ttk.Button(main_frame, text="BACKWARD",
                          command=lambda: set_mode("BACK"))
btn_left = ttk.Button(main_frame, text="LEFT",
                      command=lambda: set_mode("LEFT"))
btn_right = ttk.Button(main_frame, text="RIGHT",
                       command=lambda: set_mode("RIGHT"))
btn_stop = ttk.Button(main_frame, text="STOP",
                      command=lambda: set_mode("STOP"))

btn_forward.grid(row=2, column=1, sticky="ew", pady=5)
btn_left.grid(row=3, column=0, sticky="ew", pady=5)
btn_stop.grid(row=3, column=1, sticky="ew", pady=5)
btn_right.grid(row=3, column=2, sticky="ew", pady=5)
btn_backward.grid(row=4, column=1, sticky="ew", pady=5)

mode_label = ttk.Label(main_frame, textvariable=mode_var)
mode_label.grid(row=5, column=0, columnspan=3, sticky="w", pady=5)

log_label = ttk.Label(main_frame, textvariable=last_log_label, wraplength=380)
log_label.grid(row=6, column=0, columnspan=3, sticky="w", pady=10)

def on_close():
    try:
        send_command("STOP")
    except:
        pass
    try:
        ser.close()
    except:
        pass
    try:
        log_file.close()
    except:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

root.after(100, compute_and_send)
root.after(50, poll_serial)
root.after(100, update_speed_label)

root.mainloop()
