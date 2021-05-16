import os
import numpy as np
from transformation import Trans3D
from chessboard_pose_estimation import *
#from chessboard_state_detection import 

pose = np.array([[-0.99995239, -0.00646971, 0.0073055, 0.30789977],
[-0.00666603, 0.99960843, -0.02717619, -0.70899765],
[-0.00712682, -0.0272236, -0.99960396, 0.15277624],
[ 0.,         0.,         0.,         1.        ]])
x = pose[:3,0]
x_desire = x.copy()
x_desire[-1] = 0
unit_vector_1 = x / np.linalg.norm(x)
unit_vector_2 = x_desire / np.linalg.norm(x_desire)
dot_product = np.dot(unit_vector_1, unit_vector_2)
angle = np.arccos(dot_product)
print(angle)
rotation = Trans3D.from_angaxis(np.array([0,angle,0]))
print(rotation*Trans3D.from_tfmatrix(pose))
