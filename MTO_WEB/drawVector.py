import math
import tkinter as tk
from tkinter import ttk  # For themed widgets (optional, but recommended)
from tkinter import messagebox  # For message boxes


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


    def update_dir(self, alpha, beta, gamma):
        # calculate the vector
        #with data_lock:
            #alpha = gyrodata["alpha"]
            #beta = gyrodata["beta"]
            #gamma = gyrodata["gamma"]

        #gamma /= 180 / (2 * math.pi)
        gamma /= 45 
        beta /= 45
        #alpha /= 45

        # draw the vector
        ## draw the gamma
        self.canvas.delete("all")
        self.draw_dir(math.radians(alpha)+(math.pi/2), 1, "yellow")
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

def run_tkinter():
    # Create Tkinter window
    root = tk.Tk()
    app = tkinter_gui(root)
    root.mainloop()
