import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
from collections import deque
from matplotlib import pyplot as plt
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

lc_serial = None
ble_serial = None
lc_running = False
ble_running = False
lc_buffer = deque(maxlen=500)   # Store (input, output) tuples
ble_buffer = deque(maxlen=500)  # Store selected FSR sensor readings

adaptation_enabled = False
adaptation_window = 50
adaptation_rate = 0.05
Q = 1.0
R = 0.001
current_kp = 0.4
current_ki = 0.0
current_kd = 0.0
setpoint_value = 4000.0

def cost_function(errors, controls, Q, R):
    errors = np.array(errors)
    controls = np.array(controls)
    return np.sum(Q * errors**2 + R * controls**2)

def send_pid_to_arduino(kp, ki, kd):
    if lc_serial and lc_serial.is_open:
        cmd = f"PID,{kp},{ki},{kd}\n"
        lc_serial.write(cmd.encode())
        output_text1.insert(tk.END, f"Sent: {cmd}")
        output_text1.see(tk.END)

def send_setpoint_to_arduino(sp):
    if lc_serial and lc_serial.is_open:
        cmd = f"SET,{sp}\n"
        lc_serial.write(cmd.encode())
        output_text1.insert(tk.END, f"Sent: {cmd}")
        output_text1.see(tk.END)

def adaptation_loop():
    global current_kp, current_ki, current_kd
    if not adaptation_enabled:
        return

    if len(lc_buffer) >= adaptation_window:
        # Extract recent window
        recent = list(lc_buffer)[-adaptation_window:]
        inputs = [x[0] for x in recent]
        outputs = [x[1] for x in recent]
        errors = [setpoint_value - i for i in inputs]
        cost_now = cost_function(errors, outputs, Q, R)

        # Try increasing Kp
        test_kp = current_kp + adaptation_rate
        send_pid_to_arduino(test_kp, current_ki, current_kd)
        # Wait for new data to accumulate
        root.after(adaptation_window * 20, lambda: evaluate_kp(test_kp, cost_now, direction="up"))
    else:
        root.after(1000, adaptation_loop)

def evaluate_kp(test_kp, cost_prev, direction):
    global current_kp, current_ki, current_kd
    if len(lc_buffer) >= adaptation_window:
        recent = list(lc_buffer)[-adaptation_window:]
        inputs = [x[0] for x in recent]
        outputs = [x[1] for x in recent]
        errors = [setpoint_value - i for i in inputs]
        cost_test = cost_function(errors, outputs, Q, R)

        if cost_test < cost_prev:
            current_kp = test_kp
            send_pid_to_arduino(current_kp, current_ki, current_kd)
            output_text1.insert(tk.END, f"Adapted Kp to {current_kp:.3f}\n")
            output_text1.see(tk.END)
            # Continue adaptation
            root.after(1000, adaptation_loop)
        else:
            # Try decreasing Kp if increasing didn't help
            if direction == "up":
                test_kp = current_kp - adaptation_rate
                send_pid_to_arduino(test_kp, current_ki, current_kd)
                root.after(adaptation_window * 20, lambda: evaluate_kp(test_kp, cost_prev, direction="down"))
            else:
                # No improvement, revert to original
                send_pid_to_arduino(current_kp, current_ki, current_kd)
                root.after(1000, adaptation_loop)
    else:
        root.after(1000, adaptation_loop)

def toggle_adaptation():
    global adaptation_enabled
    adaptation_enabled = not adaptation_enabled
    if adaptation_enabled:
        adaptation_button.config(text="Stop Adaptation")
        root.after(1000, adaptation_loop)
    else:
        adaptation_button.config(text="Start Adaptation")

def list_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]

def open_lc_serial(port, baud=9600):
    global lc_serial, lc_running
    try:
        lc_serial = serial.Serial(port, baud, timeout=1)
        lc_running = True
        threading.Thread(target=read_lc_serial, daemon=True).start()
        return True
    except Exception as e:
        messagebox.showerror("Serial Error (LC)", str(e))
        return False

def open_ble_serial(port, baud=115200):
    global ble_serial, ble_running
    try:
        ble_serial = serial.Serial(port, baud, timeout=1)
        ble_running = True
        threading.Thread(target=read_ble_serial, daemon=True).start()
        return True
    except Exception as e:
        messagebox.showerror("Serial Error (BLE)", str(e))
        return False

def close_lc_serial():
    global lc_serial, lc_running
    lc_running = False
    if lc_serial and lc_serial.is_open:
        lc_serial.close()

def close_ble_serial():
    global ble_serial, ble_running
    ble_running = False
    if ble_serial and ble_serial.is_open:
        ble_serial.close()

def read_lc_serial():
    while lc_running and lc_serial and lc_serial.is_open:
        try:
            line = lc_serial.readline().decode(errors='ignore').strip()
            if line:
                output_text1.after(0, lambda l=line: (output_text1.insert(tk.END, l + '\n'), output_text1.see(tk.END)))
                try:
                    parts = line.split(",")
                    if len(parts) == 2:
                        input_val = float(parts[0])
                        output_val = float(parts[1])
                        lc_buffer.append((input_val, output_val))
                    else:
                        value = float(line)
                        lc_buffer.append((value, 0))
                except ValueError:
                    pass
        except Exception:
            break

def read_ble_serial():
    mode = mode_var.get()
    while ble_running and ble_serial and ble_serial.is_open:
        try:
            line = ble_serial.readline().decode(errors='ignore').strip()
            if line:
                output_text2.after(0, lambda l=line: (output_text2.insert(tk.END, l + '\n'), output_text2.see(tk.END)))
                try:
                    if mode == "multi":
                        cleaned_line = line.strip().rstrip('_')
                        values = [float(x) for x in cleaned_line.split("_")]
                        selected_idx = int(sensor_var.get().split()[-1]) - 1
                        if 0 <= selected_idx < len(values):
                            ble_buffer.append(values[selected_idx])
                    else:
                        value = float(line)
                        ble_buffer.append(value)
                except ValueError:
                    pass
        except Exception:
            break

def refresh_ports():
    ports = list_serial_ports()
    lc_port_combo['values'] = ports
    ble_port_combo['values'] = ports
    if ports:
        lc_port_combo.current(0)
        ble_port_combo.current(0)

def connect_lc():
    port = lc_port_combo.get()
    if open_lc_serial(port):
        connect_button1.config(state=tk.DISABLED)
        disconnect_button1.config(state=tk.NORMAL)
        status_label1.config(text="Connected", fg="green")

def disconnect_lc():
    close_lc_serial()
    connect_button1.config(state=tk.NORMAL)
    disconnect_button1.config(state=tk.DISABLED)
    status_label1.config(text="Disconnected", fg="red")

def connect_ble():
    port = ble_port_combo.get()
    if open_ble_serial(port):
        connect_button2.config(state=tk.DISABLED)
        disconnect_button2.config(state=tk.NORMAL)
        status_label2.config(text="Connected", fg="green")

def disconnect_ble():
    close_ble_serial()
    connect_button2.config(state=tk.NORMAL)
    disconnect_button2.config(state=tk.DISABLED)
    status_label2.config(text="Disconnected", fg="red")

def send_pid():
    global current_kp, current_ki, current_kd
    if lc_serial and lc_serial.is_open:
        try:
            kp = float(kp_entry.get())
            ki = float(ki_entry.get())
            kd = float(kd_entry.get())
            current_kp, current_ki, current_kd = kp, ki, kd
            cmd = f"PID,{kp},{ki},{kd}\n"
            lc_serial.write(cmd.encode())
            output_text1.insert(tk.END, f"Sent: {cmd}")
            output_text1.see(tk.END)
        except Exception as e:
            messagebox.showerror("Error", str(e))

def send_setpoint():
    global setpoint_value
    if lc_serial and lc_serial.is_open:
        try:
            setpoint = float(setpoint_entry.get())
            setpoint_value = setpoint
            cmd = f"SET,{setpoint}\n"
            lc_serial.write(cmd.encode())
            output_text1.insert(tk.END, f"Sent: {cmd}")
            output_text1.see(tk.END)
        except Exception as e:
            messagebox.showerror("Error", str(e))

def save_data():
    if not lc_buffer or not ble_buffer:
        messagebox.showinfo("No Data", "No data to save!")
        return
    filepath = filedialog.asksaveasfilename(defaultextension=".csv")
    if filepath:
        with open(filepath, "w") as f:
            f.write("LC_Input,FSR\n")
            n = min(len(lc_buffer), len(ble_buffer))
            for i in range(n):
                inp, outp = lc_buffer[-n + i]
                fsr = ble_buffer[-n + i]
                f.write(f"{inp},{fsr}\n")
        messagebox.showinfo("Saved", f"Data saved to {filepath}")

def plot_data():
    import matplotlib.pyplot as plt
    if not lc_buffer or not ble_buffer:
        messagebox.showinfo("No Data", "No data to plot!")
        return
    n = min(len(lc_buffer), len(ble_buffer))
    inputs = [lc_buffer[-n + i][0] for i in range(n)]
    outputs = [lc_buffer[-n + i][1] for i in range(n)]
    fsr_vals = [ble_buffer[-n + i] for i in range(n)]
    plt.figure()
    plt.plot(inputs, label='LC Input')
    plt.plot(outputs, label='LC Output')
    plt.plot(fsr_vals, label=f'FSR {sensor_var.get() if mode_var.get()=="multi" else "Single"}')
    plt.legend()
    plt.title("Data")
    plt.xlabel("Sample")
    plt.ylabel("Value")
    plt.show()

def plot_calibration_curve():
    if not lc_buffer or not ble_buffer:
        messagebox.showinfo("No Data", "No data to plot calibration curve!")
        return
    n = min(len(lc_buffer), len(ble_buffer))
    lc_inputs = np.array([lc_buffer[-n + i][0] for i in range(n)])
    fsr_vals = np.array([ble_buffer[-n + i] for i in range(n)])

    best_degree = 1
    best_error = float('inf')
    best_coeffs = None

    for degree in range(1, 6):
        coeffs = np.polyfit(fsr_vals, lc_inputs, degree)
        fit_fn = np.poly1d(coeffs)
        error = np.sum((lc_inputs - fit_fn(fsr_vals))**2)
        if error < best_error:
            best_error = error
            best_degree = degree
            best_coeffs = coeffs

    fit_fn = np.poly1d(best_coeffs) # type: ignore

    plt.figure()
    plt.scatter(fsr_vals, lc_inputs, label="Data Points")
    plt.plot(fsr_vals, fit_fn(fsr_vals), color='red',
             label=f"Best Fit Degree {best_degree}")
    plt.title("Calibration Curve: FSR vs LC_Input")
    plt.xlabel("FSR Sensor Reading")
    plt.ylabel("Load Cell Input")
    plt.legend()
    plt.grid(True)
    plt.show()

def sensor_selected(event=None):
    ble_buffer.clear()

def mode_changed():
    mode = mode_var.get()
    if mode == "multi":
        sensor_combo.grid(row=5, column=1, sticky=tk.W)
    else:
        sensor_combo.grid_remove()
    ble_buffer.clear()

root = tk.Tk()
root.title("Load Cell & BLE FSR Calibration GUI")

logo1 = Image.open("iitlogo.png").resize((80, 80))
logo2 = Image.open("excelsior.jpg").resize((80, 80))
logo1_img = ImageTk.PhotoImage(logo1)
logo2_img = ImageTk.PhotoImage(logo2)
root.logo_images = [logo1_img, logo2_img]  # type: ignore

# Create a frame for the logos and title
logo_frame = ttk.Frame(root)
logo_frame.grid(row=0, column=0, columnspan=7, sticky="ew", pady=(10,0))

# Place first logo
logo1_label = tk.Label(logo_frame, image=logo1_img)
logo1_label.pack(side=tk.LEFT, padx=(0,10))

# Place center text
title_label = tk.Label(
    logo_frame,
    text="UTM FOR SMART INSOLE CALIBRATION",
    font=("Arial", 20, "bold")
)
title_label.pack(side=tk.LEFT, padx=50)

# Place second logo
logo2_label = tk.Label(logo_frame, image=logo2_img)
logo2_label.pack(side=tk.LEFT, padx=(10,0))

# Now place your mainframe below the logo_frame
mainframe = ttk.Frame(root, padding="10")
mainframe.grid(row=1, column=0, sticky=(tk.N, tk.W, tk.E, tk.S)) # type: ignore

mode_var = tk.StringVar(value="multi")
ttk.Label(mainframe, text="Mode:").grid(row=0, column=0, sticky=tk.W)
mode_multi = ttk.Radiobutton(mainframe, text="BLE Multi-Sensor", variable=mode_var, value="multi", command=mode_changed)
mode_single = ttk.Radiobutton(mainframe, text="Single FSR", variable=mode_var, value="single", command=mode_changed)
mode_multi.grid(row=0, column=1, sticky=tk.W)
mode_single.grid(row=0, column=2, sticky=tk.W)

ttk.Label(mainframe, text="LC:").grid(row=1, column=0, sticky=tk.W)
lc_port_combo = ttk.Combobox(mainframe, width=15)
lc_port_combo.grid(row=1, column=1, sticky=tk.W)
refresh_button1 = ttk.Button(mainframe, text="Refresh", command=refresh_ports)
refresh_button1.grid(row=1, column=2, sticky=tk.W)
connect_button1 = ttk.Button(mainframe, text="Connect", command=connect_lc)
connect_button1.grid(row=1, column=3, sticky=tk.W)
disconnect_button1 = ttk.Button(mainframe, text="Disconnect", command=disconnect_lc, state=tk.DISABLED)
disconnect_button1.grid(row=1, column=4, sticky=tk.W)
status_label1 = tk.Label(mainframe, text="Disconnected", fg="red")
status_label1.grid(row=1, column=5, sticky=tk.W)

ttk.Label(mainframe, text="BLE:").grid(row=2, column=0, sticky=tk.W)
ble_port_combo = ttk.Combobox(mainframe, width=15)
ble_port_combo.grid(row=2, column=1, sticky=tk.W)
refresh_button2 = ttk.Button(mainframe, text="Refresh", command=refresh_ports)
refresh_button2.grid(row=2, column=2, sticky=tk.W)
connect_button2 = ttk.Button(mainframe, text="Connect", command=connect_ble)
connect_button2.grid(row=2, column=3, sticky=tk.W)
disconnect_button2 = ttk.Button(mainframe, text="Disconnect", command=disconnect_ble, state=tk.DISABLED)
disconnect_button2.grid(row=2, column=4, sticky=tk.W)
status_label2 = tk.Label(mainframe, text="Disconnected", fg="red")
status_label2.grid(row=2, column=5, sticky=tk.W)

ttk.Label(mainframe, text="Kp:").grid(row=3, column=0, sticky=tk.W)
kp_entry = ttk.Entry(mainframe, width=7)
kp_entry.insert(0, "0.4")
kp_entry.grid(row=3, column=1, sticky=tk.W)
ttk.Label(mainframe, text="Ki:").grid(row=3, column=2, sticky=tk.W)
ki_entry = ttk.Entry(mainframe, width=7)
ki_entry.insert(0, "0")
ki_entry.grid(row=3, column=3, sticky=tk.W)
ttk.Label(mainframe, text="Kd:").grid(row=3, column=4, sticky=tk.W)
kd_entry = ttk.Entry(mainframe, width=7)
kd_entry.insert(0, "0")
kd_entry.grid(row=3, column=5, sticky=tk.W)
pid_button = ttk.Button(mainframe, text="Send PID", command=send_pid)
pid_button.grid(row=3, column=6, sticky=tk.W)

ttk.Label(mainframe, text="Setpoint:").grid(row=4, column=0, sticky=tk.W)
setpoint_entry = ttk.Entry(mainframe, width=10)
setpoint_entry.insert(0, "4000")
setpoint_entry.grid(row=4, column=1, sticky=tk.W)
setpoint_button = ttk.Button(mainframe, text="Send Setpoint", command=send_setpoint)
setpoint_button.grid(row=4, column=2, sticky=tk.W)

adaptation_button = ttk.Button(mainframe, text="Start Adaptation", command=toggle_adaptation)
adaptation_button.grid(row=4, column=3, sticky=tk.W)

ttk.Label(mainframe, text="FSR Sensor:").grid(row=5, column=0, sticky=tk.W)
sensor_var = tk.StringVar(value="Sensor 1")
sensor_combo = ttk.Combobox(mainframe, textvariable=sensor_var, values=[f"Sensor {i+1}" for i in range(16)], width=10, state="readonly")
sensor_combo.grid(row=5, column=1, sticky=tk.W)
sensor_combo.bind("<<ComboboxSelected>>", sensor_selected)

save_button = ttk.Button(mainframe, text="Save Data", command=save_data)
save_button.grid(row=6, column=0, sticky=tk.W)
plot_button = ttk.Button(mainframe, text="Plot Data", command=plot_data)
plot_button.grid(row=6, column=1, sticky=tk.W)
calib_button = ttk.Button(mainframe, text="Plot Calibration", command=plot_calibration_curve)
calib_button.grid(row=6, column=2, sticky=tk.W)

output_text1 = tk.Text(mainframe, height=6, width=40)
output_text1.grid(row=7, column=0, columnspan=3, pady=5)
output_text2 = tk.Text(mainframe, height=6, width=40)
output_text2.grid(row=7, column=3, columnspan=4, pady=5)

fig = Figure(figsize=(8, 4), dpi=100)
ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)
line1, = ax1.plot([], [], label='LC Input')
line2, = ax1.plot([], [], label='LC Output')
ax1.legend()
ax1.set_title("Load Cell")
ax1.set_xlabel("Sample")
ax1.set_ylabel("Value")
line3, = ax2.plot([], [], label='FSR')
ax2.legend()
ax2.set_title("FSR Sensor")
ax2.set_xlabel("Sample")
ax2.set_ylabel("Value")
fig.subplots_adjust(bottom=0.18)

canvas = FigureCanvasTkAgg(fig, master=mainframe)
canvas.get_tk_widget().grid(row=8, column=0, columnspan=7, pady=10)

def update_plot():
    if lc_buffer:
        inputs = [x[0] for x in lc_buffer]
        outputs = [x[1] for x in lc_buffer]
        line1.set_data(range(len(inputs)), inputs)
        line2.set_data(range(len(outputs)), outputs)
        ax1.relim()
        ax1.autoscale_view()
    else:
        line1.set_data([], [])
        line2.set_data([], [])
    if ble_buffer:
        fsr_vals = list(ble_buffer)
        line3.set_data(range(len(fsr_vals)), fsr_vals)
        ax2.relim()
        ax2.autoscale_view()
    else:
        line3.set_data([], [])
    canvas.draw()
    root.after(500, update_plot)

update_plot()
mode_changed()
refresh_ports()
root.mainloop()