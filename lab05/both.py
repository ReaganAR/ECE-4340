from forward import formain
from reverse import revmain

import numpy as np

thetasboth = [30.0,30,30,30,30,30]
Tmatrix = np.identity(4)

print("Forward Kinematics from ", thetasboth)
thetasboth = revmain(formain(thetasboth))

thetaform = thetasboth.copy()
indice = 0
for f in thetaform:
        thetaform[indice] = format(f, '.3f') 
        indice = indice + 1
print("Inv. kinematics provides", thetaform)
formain(thetasboth)