import numpy as np
from scipy.special import comb

def bernstein_poly(i, n, t):
    return comb(n, i) * ( t**(n - i) ) * (1 - t)**i

def bezier_curve(points, nTimes=1000):
    nPoints = len(points)
    inputPoints = np.array([p[0] for p in points])
    outputPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    input_vals = np.dot(inputPoints, polynomial_array)
    output_vals = np.dot(outputPoints, polynomial_array)

    return input_vals, output_vals

bezier_points = [
    (-1.206, -2),
    (-0.845, 1.2),
    (0.845, -1.2),
    (1.206, 2)
]

#desmos link if anyone wants to tweak the curve: https://www.desmos.com/calculator/x0bo7r5ncj

def output_bezier(input_value):
    input,output = bezier_curve(bezier_points)
    index = np.argmin(np.absolute(input - input_value))
    
    return output[index]
