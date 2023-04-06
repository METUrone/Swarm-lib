import numpy as np
from scipy.linalg import lu_factor, lu_solve
import cv2 as cv

class Camera:
	"""
	Camera class to keep constant values and calculate coordinates

	:param np.ndarray intrinsics_old: 3x3 numpy array obtained from camera calibration
	:param np.ndarray intrinsics_new: 3x3 numpy array obtained from OptimalNewCameraMatrix method of opencv
	:param np.ndarray dist: numpy vector of distortion coefficients
	"""	
	def __init__(self, intrinsics_old, intrinsics_new, dist, roi):
		
		self.intrinsics_old = intrinsics_old
		self.intrinsics_new = intrinsics_new
		self.dist = dist
		
		#focal lenghts of camera
		self.f_x = self.intrinsics_new[0][0]
		self.f_y = self.intrinsics_new[1][1]
		
		#offset parameters of camera
		self.c_x = intrinsics_new[0][2]
		self.c_y = intrinsics_new[1][2]

		#Region of interest, (x, y, w, h)
		self.roi = roi


	def find_coord(self, GPS_x, GPS_Y, GPS_Z, img_x, img_y, roll, pitch, yaw) -> tuple:
		"""
		Finds World coordinate system cordinates of a point in image
		:param int GPS_x: X coordinate of the camera in World Coordinate system
		:param int GPS_y: Y coordinate of the camera in World Coordinate system
		:param int GPS_z: Z coordinate of the camera in World Coordinate system
		:param int img_x: X coordinate of the chosen point in image plane
		:param int img_y: Y coordinate of the chosen point in image plane
		:param float roll: Roll angle of the camera in radians
		:param float pitch: Pitch angle of the camera in radians
		:param float yaw: Yaw angle of the camera in radians
		:return: (X, Y) where X is X coordinate of the point in world coordinate system and Y is the Y coordinate of the point in world coordinate system
		:rtype: tuple
		"""

		def RotationXU(theta, radian=1):
			"""
			Returns rotational matrix around x from theta in radians. if "radian" parameter is 0 then takes angle and converts it to radian
			"""
			if not radian:
				theta = theta/180 * np.pi
			
			R_x = np.zeros((3,3), dtype=np.float64)
			
			R_x[0, 0] = 1
			R_x[1, 1] = np.cos(theta)
			R_x[2, 1] = np.sin(theta)
			R_x[1, 2] = -np.sin(theta)
			R_x[2, 2] = np.cos(theta)
			
			return R_x

		def RotationYV(theta, radian=1):
			"""
			Returns rotational matrix around y from theta in radians. if "radian" parameter is 0 then takes angle and converts it to radian
			"""
			if not radian:
				theta = theta/180 * np.pi

			R_y = np.zeros((3,3), dtype=np.float64)

			R_y[1, 1] = 1
			R_y[0, 0] = np.cos(theta)
			R_y[0, 2] = -np.sin(theta)
			R_y[2, 0] = np.sin(theta)
			R_y[2, 2] = np.cos(theta)

			return R_y

		def RotationZW(theta, radian=1):
			"""
			Returns rotational matrix around z from theta in radians. if "radian" parameter is 0 then takes angle and converts it to radian
			"""
			if not radian:
				theta = theta/180 * np.pi

			R_z = np.zeros((3,3), dtype=np.float64)

			R_z[2, 2] = 1
			R_z[0, 0] = np.cos(theta)
			R_z[0, 1] = -np.sin(theta)
			R_z[1, 0] = np.sin(theta)
			R_z[1, 1] = np.cos(theta)

			return R_z
		
		#Setup the rotaitonal matrix R
		R_Z = RotationZW(yaw),
		R_Y = RotationYV(pitch)
		R_X = RotationXU(roll)
		R = np.matmul(np.matmul(R_Z, R_Y), R_X)

		R = R[0]
		#R' = R^T because R is an orthogonal matrix
		R_inv = R.transpose()

		#set parameters of the focal_matrix*MS
		MS11=R[0][0]
		MS12=R[0][1]
		MS13=(self.c_x-img_x)/self.f_x
		MS21=R[1][0]
		MS22=R[1][1]
		MS23=(self.c_y-img_y)/self.f_y
		MS31=R[2][0]
		MS32=R[2][1]
		MS33=-1

		MS=np.array([[MS11,MS12,MS13,MS21,MS22,MS23,MS31,MS32,MS33]])
		MS_matrix=MS.reshape((3,3))

	    #R'*MS = A
		A = np.matmul(R_inv, MS_matrix)
		b = np.array([GPS_x, GPS_Y, GPS_Z]).transpose()

	    
	    #Solving problem with LU decomposition from scipy
		lu, piv = lu_factor(A)
		x = lu_solve((lu, piv), b)
	    
		return (int(x[0]), int(x[1]))

	def frame_undistort(self, frame):
		undistorted = cv.undistort(frame,self.intrinsics_old,self.dist,None,self.intrinsics_new)
		x, y, w, h = self.roi
		return undistorted[y:y+h, x:x+w]