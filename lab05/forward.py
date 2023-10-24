import numpy as np
import math

#thetas = [0, 0, 0, 0, 0, 0]
thetas = [30, 30, 30, 30, 30, 30]
thetas = np.radians(thetas)

dh_array = np.array([[thetas[0], 0, 0, -math.pi/2],
                     [thetas[1], 125.4125, 203.2, 0],
                     [thetas[2], 0, 0, math.pi/2],
                     [thetas[3], 203.2, 0, -math.pi/2],
                     [thetas[4], 0, 0, math.pi/2],
                     [thetas[5], 55.9308, 0, 0]])

def A_matrix(theta, d, a, alpha):
    matrix = np.array([[math.cos(theta), -math.cos(alpha) * math.sin(theta), math.sin(alpha) * math.sin(theta), a * math.cos(theta)],
                       [math.sin(theta), math.cos(alpha) * math.cos(theta), -math.sin(alpha) * math.cos(theta), a * math.sin(theta)],
                       [0, math.sin(alpha), math.cos(alpha), d],
                       [0,0,0,1]])
    return matrix

def calc_T():
    matrix = np.identity(4)

    for i in range(6): # int i = 0; i < 6; i++
        (theta, d, a, alpha) = tuple(dh_array[i])
        Amatrix = A_matrix(theta, d, a, alpha)
        matrix = np.dot(matrix, Amatrix)

    return matrix

def T_to_OAT(T_matrix):
    tool = np.degrees(math.atan2(T_matrix[2][1], -T_matrix[2][0]))
    orientation = math.atan2(T_matrix[0][2], -T_matrix[1][2])
    approach = np.degrees(math.atan2(-T_matrix[2][2], T_matrix[0][2] / math.sin(orientation)))

    orientation = np.degrees(orientation)

    return orientation, approach, tool

Tmatrix = calc_T()
x = Tmatrix[0][3]
y = Tmatrix[1][3]
z = Tmatrix[2][3]

print("OAT:")
print(T_to_OAT(Tmatrix))

print("XYZ:")
print(x, y, z)