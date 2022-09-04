#!/usr/bin/env python3
import time
import numpy as np
from artificial_potential_field import ArtificialPotentialField


apf = ArtificialPotentialField()
apf.form_via_potential_field(2)
print("*"*100)
apf.go([-15, -15, 0])