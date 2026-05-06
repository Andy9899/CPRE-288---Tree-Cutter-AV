# Description: Display using a polar plot the distance measurements collected by a CyBot 180 degree 
# sensor scan at 4 degree increments from 0 to 180 degrees. Sensor data read from a text file.

# Original Code example modified by: Phillip Jones (10/02/2021), (05/15/2023)
# Original polar plot code example from matplot: https://matplotlib.org/stable/gallery/pie_and_polar_charts/polar_demo.html

# Useful matplotlib tutorial: https://matplotlib.org/stable/tutorials/introductory/pyplot.html
# Useful best practices Quick Start: https://matplotlib.org/stable/tutorials/introductory/quick_start.html
# General Python Reference/Tutorials: https://www.w3schools.com/python/ 

# Quick YouTube Overviews (See above links as primary resources for additional details): 
# - Quick Polar Plot (subplot) Overview: https://www.youtube.com/watch?v=pb-pZtvkGPM
# - Quick subplots Overview : https://www.youtube.com/watch?v=Tqph7_qMujk
#Overwrite 'w' and read is 'r'

#OPEN COMMAND PROMPT BEFORE USE
#pip install mplcursors
#pip install pynput 
#pip install pywin32

#Import/Include useful math and plotting functions
import numpy as np
import matplotlib.pyplot as plt
import os  # import function for finding absolute path to this python script
import time #used for delay
import mplcursors
import tkinter as tk
from pynput.keyboard import Controller
import win32gui
import win32api
import win32con

running = False
last_event_time = 0

#Takes the file and clears it from the start_line backwards
def clear_file(filename, start_line):
    
    try:
        with open(filename, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        # Keep only the lines after the given line
        remaining_lines = lines[start_line:]

        with open(filename, 'w', encoding='utf-8') as file:
            file.writelines(remaining_lines)
        
    except FileNotFoundError:
        print("Error: File not found.")
    except PermissionError:
        print("Error: Permission denied.")

# A little python magic to make it more convient for you to ajust where you want the data file to live
# Link for more info: https://towardsthecloud.com/get-relative-path-python 
absolute_path = os.path.dirname(__file__) # Absoult path to this python script
relative_path = "./"   # Path to sensor data file relative to this python script (./ means data file is in the same directory as this python script
full_path = os.path.join(absolute_path, relative_path) # Full path to sensor data file
filename = 'scan1.txt' 

#Allows plot to get updates countinously
plt.ion()

# Create a polar plot once
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}) # One subplot of type polar

status_text = fig.text(
    0.02, 0.95,
    "Status: Ready",
    fontsize=12,
    color="red",
    ha="left",
    va="top"
)
alert_status = ""
move_status = ""

def refresh_status():
    status_text.set_text(
        "Status:\n"
        + ("Alert: " + alert_status + "\n" if alert_status else "")
        + ("Move: " + move_status if move_status else "")
    )

def send_key(char):
    hwnd = win32gui.FindWindow(None, "192.168.1.1 - PuTTY")  # exact window title
    if hwnd:
        win32api.SendMessage(hwnd, win32con.WM_CHAR, ord(char), 0)

def show_wasd():
    wasd_frame.pack()

def hide_wasd():
    wasd_frame.pack_forget()
    
def show_OG():
    OG_frame.pack()

def hide_OG():
    OG_frame.pack_forget()

def move_forward():
    send_key('w')

def move_backward():
    send_key('s')

def turn_left():
    send_key('a')

def turn_right():
    send_key('d')

def new_scan():
    send_key('r')


def start_program(event=None):
    global running
    running = True
    
    # Find PuTTY window
    hwnd = win32gui.FindWindow(None, "192.168.1.1 - PuTTY")  # exact window title
    if hwnd:
        # send 'm' keydown and keyup messages
        win32api.SendMessage(hwnd, win32con.WM_CHAR, ord('m'), 0)
    
    hide_wasd()
    show_OG()

def restart_program(event=None):
    global running
    running = True
    
    # Find PuTTY window
    hwnd = win32gui.FindWindow(None, "192.168.1.1 - PuTTY")  # exact window title
    if hwnd:
        # send 'm' keydown and keyup messages
        win32api.SendMessage(hwnd, win32con.WM_CHAR, ord('g'), 0)
    
    hide_wasd()
    show_OG()

def stop_program(event=None):
    global running
    running = False
    hwnd = win32gui.FindWindow(None, "192.168.1.1 - PuTTY")  # exact window title
    if hwnd:
        # send 'p' keydown and keyup messages
        win32api.SendMessage(hwnd, win32con.WM_CHAR, ord('p'), 0)
    
    show_wasd()
    hide_OG()
        
        
#update the plot forever
def update_plot():
    global last_event_time, alert_status, move_status
    
    # angle_degrees: a vector (i.e., array of numbers) for which each element is an angle at which the sensor makes a distance measurement.
    # Units: degrees
    angle_degrees = [] # Initially empty

    #Used for object detetion, Initially empty
    object_angles = []
    object_widths = []
    object_distances = []
    object_ids = []
        # -----------------------------------------------
    try:
        # Open file containing CyBot sensor scan from 0 - 180 degrees
        file_object = open(full_path + filename,'r') # Open the file: file_object is just a variable for the file "handler" returned by open()
        all_lines = file_object.readlines()
        cleaned_lines = []
        for line in all_lines:
            stripped = line.strip()
            if not (stripped.startswith("EVENT:") or stripped.startswith("MOVE:") or stripped.startswith("TURN:")):
                cleaned_lines.append(line)

        if cleaned_lines != all_lines:
            with open(full_path + filename, 'w', encoding='utf-8') as file:
                file.writelines(cleaned_lines)

        # Find the last scan header in the file
        last_header_index = -1
        for index, line in enumerate(all_lines):
            if line.startswith("Angle"):
                last_header_index = index

        # Keep only the newest scan
        if last_header_index != -1:
            file_header = all_lines[last_header_index]
            file_data = all_lines[last_header_index + 1:]
        else:
            file_header = ""
            file_data = all_lines
            
        file_object.close() # Important to close file one you are done with it!!

        # For each line of the file split into columns, and assign each column to a variable
        for line in file_data:

            line = line.strip()

            if not line:
                continue
            
            if line.startswith("EVENT:"):
                alert_status = line.replace("EVENT:", "")
                last_event_time = time.time()
                refresh_status()
                continue

            if line.startswith("MOVE:"):
                data = line.replace("MOVE:", "").split("_")
                
                if len(data) == 2:
                    direction, amount = data
                    move_status = f"Moved {direction} {amount} mm"
                else:
                    move_status = "Moved"
                    
                refresh_status()
                continue

            if line.startswith("TURN:"):
                data = line.replace("TURN:", "").split("_")
                
                if len(data) == 2:
                    direction, angle = data
                    move_status = f"Turned {direction} {angle}°"
                else:
                    move_status = "Turned"
                    
                refresh_status()
                continue
                
            data = line.split()
            
            # used to just read the angle
            if len(data) == 1:
                try:
                    angle = float(data[0])
                    angle_degrees.append(angle)
                except ValueError:
                    continue

            #object detected readin values
            elif len(data) >= 5:
                try:
                    angle = float(data[0])
                    obj = float(data[1])
                    obj_angle = float(data[2])
                    width = float(data[3])
                    dist = float(data[4])
                except ValueError:
                    continue

                angle_degrees.append(angle)

                object_ids.append(obj)
                object_angles.append(obj_angle)
                object_widths.append(width)
                object_distances.append(dist)

        # Convert python sequence (list of strings) into a numpy array
        angle_degrees = np.array(angle_degrees)
                                                
        
        # Used to clear previous plot
        ax.clear()

        #Plotting the objects
        if len(object_angles) > 0:

            obj_angles_rad = np.deg2rad(np.array(object_angles)) 
            obj_dist = np.array(object_distances)

            #
            scatter = ax.scatter(obj_angles_rad, obj_dist, s=150, marker='o', label="Objects")
            
            cursor = mplcursors.cursor(scatter, hover = mplcursors.HoverMode.Transient, multiple = False, highlight = True)         
            
            @cursor.connect("add")
            def on_add(sel):
                    i = sel.index
                    sel.annotation.set_text(
                    f"Obj {object_ids[i]}\n"
                    f"W={object_widths[i]}cm\n"
                    f"D={object_distances[i]}cm\n"
                    f"A={object_angles[i]}deg"
                    )
            
        

        ax.set_xlabel('Distance (cm)', fontsize = 14.0)  # Label x axis
        ax.set_ylabel('Angle (degrees)', fontsize = 14.0) # Label y axis
        ax.xaxis.set_label_coords(0.5, 0.15) 
        ax.tick_params(axis='both', which='major', labelsize=14) 
        
        ax.set_rmax(120)
        ax.set_rticks([20,40,60,80,100])
        ax.set_rlabel_position(-22.5)
        
        ax.set_thetamax(180)
        ax.set_xticks(np.arange(0,np.pi+.1,np.pi/4))
        ax.grid(True)

        # Create title for plot
        
        ax.set_title("Object Detection Plot", size=14, y=1.0, pad=-24)
        
        #Draw the graph
        plt.draw()
        plt.pause(.01)
        
        # Reset status after 2 seconds if no new event
        if alert_status and time.time() - last_event_time > 2:
            alert_status = ""
            refresh_status()
        
        
    except FileNotFoundError:
        pass
    root.after(250, update_plot)
    

root = tk.Tk()
root.title("CyBot Control")

# Button that simulates M & P key
OG_frame = tk.Frame(root)
butt = tk.Button(OG_frame, text="Start", command=start_program)
butt.grid(row=0, column=0)

stopButt = tk.Button(OG_frame, text="Manual Mode", command=stop_program)
stopButt.grid(row=1, column=0)

wasd_frame = tk.Frame(root)

w_button = tk.Button(wasd_frame, text="W", width=8, command=move_forward, repeatdelay=50, repeatinterval=50)
w_button.grid(row=0, column=1)

a_button = tk.Button(wasd_frame, text="A", width=8, command=turn_left, repeatdelay=100, repeatinterval=100)
a_button.grid(row=1, column=0)

s_button = tk.Button(wasd_frame, text="S", width=8, command=move_backward, repeatdelay=100, repeatinterval=100)
s_button.grid(row=1, column=1)

d_button = tk.Button(wasd_frame, text="D", width=8, command=turn_right, repeatdelay=100, repeatinterval=100)
d_button.grid(row=1, column=2)

g_button = tk.Button(wasd_frame, text="Restart", width=8, command=restart_program)
g_button.grid(row=0, column=3)

r_button = tk.Button(wasd_frame, text="Scan", width=8, command=new_scan)
r_button.grid(row=0, column=4)


hide_wasd()
show_OG()

# (IMPORTANT) This makes the simulated key actually do something
root.bind("<KeyPress-m>", start_program)

# (IMPORTANT) This makes the simulated key actually do something
root.bind("<KeyPress-p>", stop_program)

update_plot()
root.mainloop()
