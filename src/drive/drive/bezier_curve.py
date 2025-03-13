import numpy as np
from scipy.special import comb

def bernstein_poly(i, n, t):
    return comb(n, i) * ( t**(n - i) ) * (1 - t)**i

def bezier_curve(points, nTimes=1000):
    nPoints = len(points)
    horizontalPoints = np.array([p[0] for p in points])
    verticalPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    hor_vals = np.dot(horizontalPoints, polynomial_array)
    vert_vals = np.dot(verticalPoints, polynomial_array)

    return hor_vals, vert_vals

bezier_points = [
    (-1.206, -2),
    (-0.845, 1.2),
    (0.845, -1.2),
    (1.206, 2)
]

def vertical_axis_bezier(horizontal_value):
    horizontal,vertical = bezier_curve(bezier_points)
    index = np.argmin(np.absolute(horizontal - horizontal_value))
    
    return vertical[index]
