import subprocess
import tkinter as tk
from threading import Thread
from tkinter import messagebox, scrolledtext

proc = None  # Global variable to store the subprocess

def read_output(proc, output_widget):
    while True:
        output = proc.stdout.readline()
        if output:
            output_widget.insert(tk.END, output)
            output_widget.see(tk.END)
        elif proc.poll() is not None:
            break

def execute_command(output_widget):
    global proc
    # command = f'powershell.exe cd ~ ; cd eve_ws ; rossetup ; roslaunch eve_main eve.launch'
    command = f'powershell.exe cd ~ ; cd eden_green_robotics ; cd test_script ; python test_script.py'
    try:
        proc = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        thread = Thread(target=read_output, args=(proc, output_widget))
        thread.start()
    except Exception as e:
        messagebox.showerror("Error", str(e))

def stop_command():
    global proc
    if proc:
        proc.terminate()  # Send SIGTERM
        proc = None

app = tk.Tk()
app.title("EVE Robotic Platform")

tk.Label(app, text="Output:").pack(pady=5)

output_widget = scrolledtext.ScrolledText(app, width=60, height=20)
output_widget.pack(pady=5)

execute_button = tk.Button(app, text="Start Harvesting Loop", command=lambda: execute_command(output_widget))
execute_button.pack(pady=5)

stop_button = tk.Button(app, text="Stop Harvesting Loop", command=stop_command)
stop_button.pack(pady=5)

app.mainloop()
