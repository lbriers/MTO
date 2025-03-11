from xml.etree.ElementTree import tostring

from flask import Flask, request, jsonify
from flask import render_template
import json
import threading
import math
import tkinter as tk
from tkinter import ttk

app = Flask(__name__)

# Shared data and lock
gyrodata = {"alpha": 0, "beta": 0, "gamma": 0}
data_lock = threading.Lock()

@app.route("/")
def hello_world():
    return render_template("index.html")

@app.route('/orientationdata', methods=['POST'])
def handle_orientationdata():
    data = request.get_json()

    if data:
        alpha = data.get('alpha')
        beta = data.get('beta')
        gamma = data.get('gamma')

        #print(f"Received orientation data: Alpha={alpha}, Beta={beta}, Gamma={gamma}")
        with data_lock:
            gyrodata["alpha"] = alpha
            gyrodata["beta"] = beta
            gyrodata["gamma"] = gamma
        # Process the data (e.g., store in a database)
        # ...

        return jsonify({'status': 'success', 'message': 'Orientation data received'}), 200
    else:
        return jsonify({'status': 'error', 'message': 'No data received'}), 400

class tkinter_gui:
    def __init__(self, root):
        self.root = root
        root.title("My Tkinter Application")

        # --- Widgets ---
        # Label
        self.label = ttk.Label(root, text="Input direction visualisation")
        self.label.grid(row=0, column=0, padx=10, pady=10, sticky=tk.W)

        # Canvas for drawing
        self.canvas_width = 400
        self.canvas_height = 400
        self.canvas = tk.Canvas(root, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.grid(row=2, column=0, columnspan=2, padx=10, pady=10)

        self.update_dir()

    def update_dir(self):
        # calculate the vector
        with data_lock:
            alpha = gyrodata["alpha"]
            beta = gyrodata["beta"]
            gamma = gyrodata["gamma"]

        gamma = gamma - (sign(gamma)*180)
        gamma /= 45
        beta /= 45
        # draw the vector
        ## draw the gamma
        self.canvas.delete("all")
        self.draw_dir(math.radians(alpha), gamma, "green")
        self.draw_dir(math.radians(alpha)+(math.pi/2), beta, "red")

        self.root.after(5, self.update_dir)

    def draw_dir(self, angle, scale, color):
        """Draws a rectangle on the canvas."""
        x1 = 200
        y1 = 200
        l = 150*scale
        x2 = x1 + (l*math.cos(angle))
        y2 = y1 - (l*math.sin(angle))
        self.canvas.create_line(x1, y1, x2, y2, fill=color)

def sign(num):
    return -1 if num < 0 else 1

def run_tkinter():
    # Create Tkinter window
    root = tk.Tk()
    app = tkinter_gui(root)
    root.mainloop()

if __name__ == '__main__':
    # Start Flask server in a separate thread
    tkinter_thread = threading.Thread(target=run_tkinter)
    tkinter_thread.daemon = True  # Allow main thread to exit even if this is running
    tkinter_thread.start()

    app.run(debug=True, host='0.0.0.0', port=5000, ssl_context='adhoc')


