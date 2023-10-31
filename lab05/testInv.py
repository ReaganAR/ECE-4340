from forward import formain
from reverse import revmain

import numpy as np
import math

PosX = 30
PosY = 50
PosZ = 70

OAT = [30,30,30]

Tmatrix = np.identity(4)
Tmatrix[0][3] = PosX
Tmatrix[1][3] = PosY
Tmatrix[2][3] = PosZ

rotM = [[-math.sin(OAT[0])*math.sin(OAT[1])*math.cos(OAT[2]) + math.cos(OAT[2]),
         math.sin(OAT[0])*math.sin(OAT[1])*math.sin(OAT[2]) + math.cos(OAT[0]) * math.cos(OAT[2]),
         math.sin(OAT[0]) * math.cos(OAT[1])],
        [math.cos(OAT[0])*math.sin(OAT[1])*math.cos(OAT[2]) + math.sin(OAT[2]),
         -math.cos(OAT[0])*math.sin(OAT[1])*math.sin(OAT[2]) + math.sin(OAT[0]) * math.cos(OAT[2]),
         -math.cos(OAT[0]) * math.cos(OAT[1])],
        [-math.cos(OAT[1]) * math.cos(OAT[2]), math.cos(OAT[1]) * math.sin(OAT[2]), -math.sin(OAT[1])]]

Tmatrix[0:3, 0:3] = rotM

thetasboth = revmain(Tmatrix)
thetaform = thetasboth.copy()
indice = 0
for f in thetaform:
        thetaform[indice] = format(f, '.3f') 
        indice = indice + 1
print("Inv. kinematics provides", thetaform)
formain(thetasboth)

# def T_to_OAT(T_matrix):
#     tool = np.degrees(math.atan2(T_matrix[2][1], -T_matrix[2][0]))
#     orientation = math.atan2(T_matrix[0][2], -T_matrix[1][2])
#     approach = np.degrees(math.atan2(-T_matrix[2][2], T_matrix[0][2] / math.sin(orientation)))

#     orientation = np.degrees(orientation)

#     return [orientation, approach, tool]