import math
import tkinter as tk
from tkinter import ttk  # For themed widgets (optional, but recommended)
from tkinter import messagebox  # For message boxes

class DirectionVisualiser:
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

        # Draw a rectangle on the canvas
        self.draw_dir(math.pi/2, 0.5, "green")
        self.draw_dir(math.pi / 3, 1, "red")

    def update_dir(self, alpha, beta, gamma):
        # calculate the vector
        gamma -= 180
        gamma /= 90
        beta /= 90
        # draw the vector
        ## draw the gamma
        self.draw_dir((math.pi/2)+math.radians(alpha), gamma, "green")
        self.draw_dir(math.radians(alpha), beta, "red")
        return

    def draw_dir(self, angle, scale, color):
        """Draws a rectangle on the canvas."""
        x1 = 200
        y1 = 200
        l = 150*scale
        x2 = x1 + (l*math.cos(angle))
        y2 = y1 - (l*math.sin(angle))
        self.canvas.create_line(x1, y1, x2, y2, fill=color)



# --- Main ---
if __name__ == "__main__":
    root = tk.Tk()
    app = MyApplication(root)
    root.mainloop()
