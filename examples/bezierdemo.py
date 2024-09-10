#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
from bezierapproxlib import BezierApproxPoint
from bezierapproxlib import bezier_approx
from bezierapproxlib import bezier_approx_get_curve_value


class MouseTrackerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("BezierApprox Demo")

        # Create a frame to hold the widgets
        self.frame = ttk.Frame(self.root)
        self.frame.pack(pady=5)

        # Label
        self.label = ttk.Label(self.frame,
                               text="Max distance:")
        self.label.pack(side=tk.LEFT, padx=5, pady=5)

        # Numeric entry field
        self.numeric_var = tk.StringVar(value="25")
        self.numeric_entry = ttk.Entry(self.frame,
                                       textvariable=self.numeric_var)
        self.numeric_entry.pack(side=tk.LEFT, padx=5, pady=5)

        # Button
        self.button = ttk.Button(self.frame, text="Recalculate",
                                 command=self.change_label_text)
        self.button.pack(side=tk.LEFT, padx=5, pady=5)

        # Check box
        self.checkbox_var = tk.BooleanVar(value=True)
        self.checkbox = ttk.Checkbutton(self.frame, text="Bezier Controls",
                                        variable=self.checkbox_var,
                                        command=self.checkbutton_clicked)
        self.checkbox.pack(side=tk.LEFT, padx=5, pady=5)

        # Canvas
        self.canvas = tk.Canvas(self.root, bg="white", width=600, height=400)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # List to store coordinates
        self.coordinates = []
        self.points = []

        # Bind mouse events
        self.canvas.bind("<B1-Motion>", self.track_mouse)
        self.canvas.bind("<ButtonRelease-1>", self.finish_tracking)

    def track_mouse(self, event):
        # Get current mouse position
        x, y = event.x, event.y
        # Save coordinates to the list
        self.coordinates.append((x, y))
        # Optionally, draw a small circle at the mouse position
        self.canvas.create_oval(x-1, y-1, x+1, y+1, fill='black')

    def checkbutton_clicked(self):
        self.redraw_points()

    def finish_tracking(self, event):
        self.points = []
        for coord in self.coordinates:
            x = coord[0]
            y = coord[1]
            # self.canvas.create_oval(x-1, y-1, x+1, y+1, fill='black')
            self.points.append(BezierApproxPoint(x, y))

        # Clear the coordinates list
        self.coordinates = []

        self.redraw_points()

    def redraw_points(self):
        self.canvas.delete("all")

        for point in self.points:
            x = point.x
            y = point.y
            self.canvas.create_oval(x-1, y-1, x+1, y+1, fill='black')

        curves = bezier_approx(self.points, float(self.numeric_var.get()))
        print(f"Controls count: {len(curves)}")
        for controls in curves:
            self.render_bezier_controls(controls)

    def change_label_text(self):
        self.redraw_points()

    def render_bezier_controls(self, controls):
        STEP = 0.05
        points = []
        t = 0.0

        while t <= 1.0001:
            point = bezier_approx_get_curve_value(controls, t)
            points.append((point.x, point.y))
            t += STEP

        print("controls result:")
        print(f"Controls P0: (x={controls.P0.x}, y={controls.P0.y})")
        print(f"Controls P1: (x={controls.P1.x}, y={controls.P1.y})")
        print(f"Controls P2: (x={controls.P2.x}, y={controls.P2.y})")
        print(f"Controls P3: (x={controls.P3.x}, y={controls.P3.y})")

        print(f"Approx First point: (x={points[0][0]}, y={points[0][1]})")
        print(f"Approx Last point: (x={points[-1][0]}, y={points[-1][1]})")

        for i in range(len(points) - 1):
            self.canvas.create_line(points[i][0], points[i][1],
                                    points[i + 1][0], points[i + 1][1],
                                    fill='red')

        if self.checkbox_var.get():
            self.canvas.create_rectangle(
                controls.P0.x - 2,
                controls.P0.y - 2,
                controls.P0.x + 2,
                controls.P0.y + 2,
                fill='green')
            self.canvas.create_rectangle(
                controls.P1.x - 2,
                controls.P1.y - 2,
                controls.P1.x + 2,
                controls.P1.y + 2,
                fill='green')
            self.canvas.create_rectangle(
                controls.P2.x - 2,
                controls.P2.y - 2,
                controls.P2.x + 2,
                controls.P2.y + 2,
                fill='green')
            self.canvas.create_rectangle(
                controls.P3.x - 2,
                controls.P3.y - 2,
                controls.P3.x + 2,
                controls.P3.y + 2,
                fill='green')

    # Function to generate points for t from 0 to 1 with step 0.1
    def generate_bezier_points(self, controls):
        points = []
        tArr = [0.0, 0.05, 0.2, 0.4, 0.6, 0.8, 0.95, 1.0]
        for t in tArr:
            print(t)
            point = bezier_approx_get_curve_value(controls, t)
            points.append((point.x, point.y))
        return points

    def render_sample_bezier(self):
        self.points = [
            BezierApproxPoint(50.00, 300.00),
            BezierApproxPoint(65.00, 240.12),
            BezierApproxPoint(110.00, 139.60),
            BezierApproxPoint(170.00, 132.80),
            BezierApproxPoint(230.00, 181.20),
            BezierApproxPoint(290.00, 186.40),
            BezierApproxPoint(335.00, 102.76),
            BezierApproxPoint(350.00, 50.00)
        ]
        self.redraw_points()


if __name__ == "__main__":
    root = tk.Tk()
    app = MouseTrackerApp(root)
    app.render_sample_bezier()
    root.mainloop()
