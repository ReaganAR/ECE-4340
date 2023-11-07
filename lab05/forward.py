import numpy as np
import math

import sys

dh_array=    [[0, 0, 0, -math.pi/2],
              [0, 125.4125, 203.2, 0],
              [0, 0, 0, math.pi/2],
              [0, 203.2, 0, -math.pi/2],
              [0, 0, 0, math.pi/2],
              [0, 55.9308, 0, 0]]

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

    return [orientation, approach, tool]

def formain(thetas):
    global dh_array
    thetas = np.radians(thetas)

    dh_array=[[thetas[0], 0, 0, -math.pi/2],
              [thetas[1], 125.4125, 203.2, 0],
              [thetas[2], 0, 0, math.pi/2],
              [thetas[3], 203.2, 0, -math.pi/2],
              [thetas[4], 0, 0, math.pi/2],
              [thetas[5], 55.9308, 0, 0]]
    
    Tmatrix = calc_T()
    x = Tmatrix[0][3]
    y = Tmatrix[1][3]
    z = Tmatrix[2][3]
   
    test= T_to_OAT(Tmatrix)
    indice = 0
    for f in test:
        test[indice] = format(f, '.3f') 
        indice = indice + 1
    print("OAT:\t", test[0], '\t', test[1], '\t', test[2])
    print("XYZ:\t", format(x, '.3f'), '\t', format(y, '.3f'), '\t', format(z, '.3f'))

    return Tmatrix

if __name__ == "__main__":
    if len(sys.argv) != 7:
        print("Usage:",sys.argv[0],"deg deg deg deg deg deg")
        sys.exit()

    input = [0,0,0,0,0,0]
    for i in range(6):
        input[i] = float(sys.argv[i + 1])
    
    formain(input)