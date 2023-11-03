import numpy as np
import math

from forward import dh_array
from forward import A_matrix

import sys

''' CONSTANTS'''
d2 = 125.4125
d4 = 203.2
d6 = 55.9308
a2 = 203.2

''' SET VARIABLES'''
thetas = [0,0,0,0,0,0]
# Tmatrix for 30,30,30,30,30,30
Tmatrix = np.zeros((4,4))

# Create a Tmatrix using the known A1, A2, and A3 (0_T_3)
def calc_T3():
    global dh_array
    global thetas

    matrix = np.identity(4)

    for i in range(3): # int i = 0; i < 6; i++
        (theta, d, a, alpha) = tuple(dh_array[i])
        Amatrix = A_matrix(thetas[i], d, a, alpha)
        matrix = np.dot(matrix, Amatrix)

    return matrix

def revmain(matrix):
    global Tmatrix, thetas

    Tmatrix = matrix
    Px = Tmatrix[0][3] - ((d6) * Tmatrix[0][2])
    Py = Tmatrix[1][3] - ((d6) * Tmatrix[1][2])
    Pz = Tmatrix[2][3] - ((d6) * Tmatrix[2][2])

    '''DETERMINE THETA 1'''
    temptop = Py * math.sqrt(Px**2 + Py**2 - d2**2) - (d2 * Px)
    tempbot = Px * math.sqrt(Px**2 + Py**2 - d2**2) + (d2 * Py)
    thetas[0] = math.atan2(temptop, tempbot)

    '''DETERMINE THETA 3'''
    rbig = ((Px**2) + (Py**2) + (Pz**2) - (a2**2) - (d2**2) - (d4**2))
    rsmall = 2 * a2 * d4
    thetas[2] = math.atan2(rbig, math.sqrt(rsmall**2 - rbig**2))

    '''DETERMINE THETA 2'''
    sintop = -Pz * (a2 + d4 * math.sin(thetas[2])) - (d4 * math.cos(thetas[2]) * math.sqrt(Px**2 + Py**2 - d2**2))
    costop = - (a2 + (d4 * math.sin(thetas[2]))) * math.sqrt(Px**2 + Py**2 - d2**2) + (d4 * math.cos(thetas[2]) * Pz)
    thetas[1] = math.atan(sintop / costop)

    '''DETERMINE THETA 4'''
    zero_T_three = calc_T3()
    three_T_six = np.dot(np.linalg.inv(zero_T_three), Tmatrix)
    thetas[3] = math.atan2(three_T_six[1][2], three_T_six[0][2])

    '''DETERMINE THETA 5'''
    thetas[4] = math.atan2(three_T_six[1][2] / math.sin(thetas[3]), three_T_six[2][2])

    '''DETERMINE THETA 6'''
    thetas[5] = math.atan2(three_T_six[2][1], -three_T_six[2][0])

    


    thetas = np.degrees(thetas)
    return thetas


if __name__ == "__main__":
    if len(sys.argv) != 7:
        print("Usage:",sys.argv[0],"x y z o a t\nWith XYZ in mm and OAT in degrees")
        sys.exit()

    inputs = [0,0,0,0,0,0]
    for i in range(6):
        inputs[i] = float(sys.argv[i + 1])
    OAT = np.radians(inputs[3:6])

    input = np.identity(4)
    input = [[-math.sin(OAT[0])*math.sin(OAT[1])*math.cos(OAT[2]) + math.cos(OAT[0])*math.sin(OAT[2]),
         math.sin(OAT[0])*math.sin(OAT[1])*math.sin(OAT[2]) + math.cos(OAT[0]) * math.cos(OAT[2]),
         math.sin(OAT[0]) * math.cos(OAT[1]),0],
        [math.cos(OAT[0])*math.sin(OAT[1])*math.cos(OAT[2]) + math.sin(OAT[0]) * math.sin(OAT[2]),
         -math.cos(OAT[0])*math.sin(OAT[1])*math.sin(OAT[2]) + math.sin(OAT[0]) * math.cos(OAT[2]),
         -math.cos(OAT[0]) * math.cos(OAT[1]),0],
        [-math.cos(OAT[1]) * math.cos(OAT[2]), math.cos(OAT[1]) * math.sin(OAT[2]), -math.sin(OAT[1]),0],
        [0,0,0,1]]

    input[0][3] = inputs[0]
    input[1][3] = inputs[1]
    input[2][3] = inputs[2]

    print(revmain(input))