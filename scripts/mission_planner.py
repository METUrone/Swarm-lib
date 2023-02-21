#!/usr/bin/env python3
import time
import numpy as np
from artificial_potential_field import ArtificialPotentialField
import os


apf = ArtificialPotentialField()
print("Takeoff")
# apf.form_3d(0.5, 3, h=0.5)

# apf.form_via_potential_field(0.7, (0.0, 0.0, -0.2))

apf.form_3d(0.5, 3, h=0.5, obj_h=1)

# apf.form_3d(0.4, 2)
time.sleep(1)

apf.go([0.5, 0, 0.0])


apf.go([0, 0, -0.5])
print("Land")

os.system("rosnode kill -a")
