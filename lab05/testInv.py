from forward import formain
from reverse import revmain

import numpy as np
import math

PosX = 300.096
PosY = 351.011
PosZ = 36.734

OAT = [134.497,-3.333,55.703]
OAT = np.radians(OAT)

Tmatrix = np.identity(4)


Tmatrix = [[-math.sin(OAT[0])*math.sin(OAT[1])*math.cos(OAT[2]) + math.cos(OAT[0])*math.sin(OAT[2]),
         math.sin(OAT[0])*math.sin(OAT[1])*math.sin(OAT[2]) + math.cos(OAT[0]) * math.cos(OAT[2]),
         math.sin(OAT[0]) * math.cos(OAT[1]),0],
        [math.cos(OAT[0])*math.sin(OAT[1])*math.cos(OAT[2]) + math.sin(OAT[0]) * math.sin(OAT[2]),
         -math.cos(OAT[0])*math.sin(OAT[1])*math.sin(OAT[2]) + math.sin(OAT[0]) * math.cos(OAT[2]),
         -math.cos(OAT[0]) * math.cos(OAT[1]),0],
        [-math.cos(OAT[1]) * math.cos(OAT[2]), math.cos(OAT[1]) * math.sin(OAT[2]), -math.sin(OAT[1]),0],
        [0,0,0,1]]

Tmatrix[0][3] = PosX
Tmatrix[1][3] = PosY
Tmatrix[2][3] = PosZ

thetasboth = revmain(Tmatrix)
thetaform = thetasboth.copy()
indice = 0
for f in thetaform:
        thetaform[indice] = format(f, '.3f') 
        indice = indice + 1
print("Inv. kinematics provides", thetaform)
formain(thetasboth)