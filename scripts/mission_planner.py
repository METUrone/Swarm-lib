#!/usr/bin/env python3
import numpy as np
from artificial_potential_field import ArtificialPotentialField

apf = ArtificialPotentialField()
apf.take_off_all_drones()
# apf.form_3d(1, "prism")
# apf.go(np.array([0, 0, 0]))
# apf.rotate(60)
# apf.form_via_potential_field(1)
apf.land_all_drones()