import numpy as np
import math

from forward import dh_array
from forward import A_matrix

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
    Px = Tmatrix[0][3]
    Py = Tmatrix[1][3]
    Pz = Tmatrix[2][3]


    '''DETERMINE THETA 1'''
    temptop = Py * math.sqrt(Px**2 + Py**2 - d2**2) - (d2 * Px)
    tempbot = Px * math.sqrt(Px**2 + Py**2 - d2**2) + (d2 * Py)
    thetas[0] = math.atan2(temptop, tempbot)

    '''DETERMINE THETA 3'''
    rbig = (Px**2 + Py**2 + Pz**2 - a2**2 - d2**2 - d4**2)
    rsmall = 2 * a2 * d4
    temptop = 2 * a2 * d4 * rbig
    tempbot = 2 * a2 * d4 * math.sqrt(rsmall**2 - rbig*2)
    thetas[2] = math.atan2(temptop, tempbot)

    '''DETERMINE THETA 2''' # SIGN ERROR SOMEWHERE, 180 deg off?
    rsmall = math.sqrt(Px**2 + Py**2 + Pz**2 - d2**2)
    sintop = -Pz * (a2 + d4 * math.sin(thetas[2])) - (d4 * math.cos(thetas[2]) * math.sqrt(Px**2 + Py**2 - d2**2))
    sin2 = sintop / (rsmall**2)
    costop = - (a2 + (d4 * math.sin(thetas[2]))) * math.sqrt(Px**2 + Py**2 - d2**2) + (d4 * math.cos(thetas[2]) * Pz)
    cos2 = costop / (rsmall**2)
    thetas[1] = math.atan2(-sin2, -cos2)

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
    input = np.array([[-5.55769053e-01, -4.29126588e-01,  7.12019053e-01,  281.917545],
                    [ 6.12139290e-01,  3.68269053e-01,  6.99759526e-01,  323.724883],
                    [-5.62500000e-01,  8.24759526e-01,  5.80127019e-02,  3.24469683],
                    [ 0,  0,  0,  1]])
    
    print(revmain(input))