#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import ctypes
import os
import platform
from ctypes import POINTER, Structure, byref, c_double, c_int


# Define the BezierApproxPoint structure
class BezierApproxPoint(Structure):
    _fields_ = [("x", c_double), ("y", c_double)]


# Define the BezierApproxCurve3Controls structure
class BezierApproxCurve3Controls(Structure):
    _fields_ = [
        ("P0", BezierApproxPoint),
        ("P1", BezierApproxPoint),
        ("P2", BezierApproxPoint),
        ("P3", BezierApproxPoint),
    ]


work_dir = os.path.dirname(os.path.realpath(__file__))
bezier_lib_path = os.path.join(work_dir, 'bezierapproxlib.dll')
if platform.system() == 'Linux':
    bezier_lib_path = os.path.join(work_dir, 'libbezierapproxlib.so')

# Load the C library
# Replace 'bezier' with the actual name of your shared library
bezier_lib = ctypes.CDLL(bezier_lib_path)

# Define the return type and argument types for
# the bezierApproxGetCurveValue function
bezier_lib.bezierApproxGetCurveValue.restype = BezierApproxPoint
bezier_lib.bezierApproxGetCurveValue.argtypes = [BezierApproxCurve3Controls,
                                                 c_double]


# Define a Python function to call the C function
def bezier_approx_get_curve_value(controls, t):
    return bezier_lib.bezierApproxGetCurveValue(controls, t)


# Define the return type and argument types for
# the bezierApproxByOneCurve function
bezier_lib.bezierApproxByOneCurve.restype = c_int
bezier_lib.bezierApproxByOneCurve.argtypes = [
    POINTER(BezierApproxPoint),
    c_int,
    c_int,
    POINTER(BezierApproxCurve3Controls)]


# Define a Python function to call the C function
def bezier_approx_by_one_curve(points, first_point_index, last_point_index):
    point_array = (BezierApproxPoint * len(points))(*points)
    controls = BezierApproxCurve3Controls()
    result = bezier_lib.bezierApproxByOneCurve(
        point_array,
        first_point_index,
        last_point_index,
        byref(controls))
    if result != 0:
        raise ValueError(f"C function returned error code: {result}")
    return controls


# Define the return type and argument types for the bezierApprox function
bezier_lib.bezierApprox.restype = c_int
bezier_lib.bezierApprox.argtypes = [
    POINTER(BezierApproxPoint),  # points array
    c_int,                       # pointsSize
    c_double,                    # precision
    POINTER(BezierApproxCurve3Controls),  # controlsBuffer
    POINTER(c_int)               # controlsBufferSize
]


# Define a Python function to call the C function
def bezier_approx(points, precision):
    points_size = len(points)
    point_array = (BezierApproxPoint * points_size)(*points)

    # We don't know how many curves will be returned,
    # so we need to allocate a buffer.
    # Let's assume the worst case where we have as many curves as points.
    controls_buffer_size = c_int(points_size)
    controls_buffer = (BezierApproxCurve3Controls * points_size)()

    # Call the C function
    result = bezier_lib.bezierApprox(
        point_array,
        points_size,
        precision,
        controls_buffer,
        byref(controls_buffer_size)
    )

    if result != 0:
        raise ValueError(f"C function returned error code: {result}")

    # Extract the curves from the controls buffer, up to controls_buffer_size
    curves = [controls_buffer[i] for i in range(controls_buffer_size.value)]
    return curves


# Example usage
if __name__ == "__main__":
    # Define the control points
    P0 = BezierApproxPoint(0.0, 0.0)
    P1 = BezierApproxPoint(1.0, 2.0)
    P2 = BezierApproxPoint(2.0, 2.0)
    P3 = BezierApproxPoint(3.0, 0.0)

    controls = BezierApproxCurve3Controls(P0, P1, P2, P3)

    t = 0.5
    point = bezier_approx_get_curve_value(controls, t)

    print("bezier_approx_get_curve_value result:")
    print(f"Point at t={t}: (x={point.x}, y={point.y})")

    # Define the points
    points = [
        BezierApproxPoint(50.00, 300.00),
        BezierApproxPoint(65.00, 240.12),
        BezierApproxPoint(110.00, 139.60),
        BezierApproxPoint(170.00, 132.80),
        BezierApproxPoint(230.00, 181.20),
        BezierApproxPoint(290.00, 186.40),
        BezierApproxPoint(335.00, 102.76),
        BezierApproxPoint(350.00, 50.00)
    ]

    first_point_index = 0
    last_point_index = len(points) - 1

    controls = bezier_approx_by_one_curve(
        points,
        first_point_index,
        last_point_index)

    print("bezier_approx_by_one_curve result:")
    print(f"Controls P0: (x={controls.P0.x}, y={controls.P0.y})")
    print(f"Controls P1: (x={controls.P1.x}, y={controls.P1.y})")
    print(f"Controls P2: (x={controls.P2.x}, y={controls.P2.y})")
    print(f"Controls P3: (x={controls.P3.x}, y={controls.P3.y})")

    precision = 5.0

    # Get the approximated curves
    curves = bezier_approx(points, precision)

    # Print the control points of each approximated curve
    for i, curve in enumerate(curves):
        print(f"Curve {i}:")
        print(f"  P0: (x={curve.P0.x}, y={curve.P0.y})")
        print(f"  P1: (x={curve.P1.x}, y={curve.P1.y})")
        print(f"  P2: (x={curve.P2.x}, y={curve.P2.y})")
        print(f"  P3: (x={curve.P3.x}, y={curve.P3.y})")
