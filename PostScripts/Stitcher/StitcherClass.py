# =============================================================================
"""
Code Information:
	Programmer: John Betancourt G.
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision & Ai Team
Merits to:
	Real-time panorama and image stitching with OpenCV
	Adrian Rosebrock 
	January 25, 2016 
	https://www.pyimagesearch.com/2016/01/25/real-time-panorama-and-image-stitc
	hing-with-opencv/
"""

# =============================================================================
from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

import numpy as np
import cv2

from Utils import get_projection_point_dst
from Utils import get_projection_point_src

# =============================================================================
def is_cv3(or_better=False):
	# grab the OpenCV major version number
	major = get_opencv_major_version()

	# check to see if we are using *at least* OpenCV 3
	if or_better:
		return major >= 3

	# otherwise we want to check for *strictly* OpenCV 3
	return major == 3

def get_opencv_major_version(lib=None):
    # if the supplied library is None, import OpenCV
    if lib is None:
        import cv2 as lib

    # return the major version number
    return int(lib.__version__.split(".")[0])

# =============================================================================
class Stitcher(Debugger):

	def __init__(self, images_dic):
		
		self.img_labels=np.sort(images_dic.keys())
		self.stitcher_labels=[]
		for idx, _ in enumerate(self.img_labels[:-1]):
			if idx==0:
				self.stitcher_labels.append("({}&{})".format(
					self.img_labels[idx], self.img_labels[idx+1]))
			else:
				self.stitcher_labels.append("({}&{})".format(
					self.stitcher_labels[-1], self.img_labels[idx+1]))
		self.stitchers=[StitcherBase(sid=cam_label) for cam_label in self.stitcher_labels]
			
	def calibrate_stitcher(self, images_dic):

		# check to see if we are using OpenCV 3.X contrib
		if is_cv3(or_better=False):
			try:
				cv2.xfeatures2d.SIFT_create()
			except:
				self.debugger(DEBUG_LEVEL_0, "OpenCV is not a contrib version, check for the module xfeatures2d", log_type="err")
				return

		# Calibrate every stitcher with images list
		for idx, _ in enumerate(self.img_labels[:-1]):
			self.debugger(DEBUG_LEVEL_0, "[STITCHER]: Calibrating stitcher {}".format(
				self.stitchers[idx].sid), log_type="info")
			images=(images_dic[self.img_labels[idx]], images_dic[self.img_labels[
				idx+1]]) if idx==0 else (img_result, images_dic[self.img_labels[idx+1]])
		
			self.stitchers[idx].calibrate(images=images)
			cv2.imshow("img_result", self.stitchers[idx].stitch(images=images))

		# Print status of stitchers
		for stitcher in self.stitchers:
			self.debugger(DEBUG_LEVEL_0, "[STITCHER]: {}".format(stitcher), 
				log_type="err" if stitcher.status is None else "info")

	def stitch(self):
		pass

	def save_stitcher(self):
		pass

	def load_stitcher(self):

		for stitcher in self.stitchers:
			self.debugger(DEBUG_LEVEL_0, "[STITCHER]: {}".format(stitcher), 
				log_type="err" if stitcher.status is None else "info")

 	def __str__(self):
		return ""
		
# =============================================================================
class StitcherBase(Debugger):

	def __init__(self, sid=None):

		self.sid = sid # Stitcher identifier

		# Stitcher properties
		self.cachedBH = None # Rotation/translation inverse matrix of image B
		self.cachedBINVH = None # Rotation/translation inverse matrix of image B
		self.Bpts= None # Conners of image B in stitcher result

		self.cachedAH = None # Rotation/translation inverse matrix of image A
		self.cachedAINVH = None # Rotation/translation inverse matrix of image A
		self.Apts= None # Conners of image A in stitcher result
	
		self.matches = None # Matches point between image A and B
		self.status = None # Status of matches between image A and B
		
		self.ABSize=None # Size of stitcher result

	def stitch(self, images):
		
		# unpack the images
		(imageB, imageA) = images

		# If stitcher is calibrated
		if self.cachedAH is not None:
			# apply a perspective transform to stitch the images together
			# using the cached homography matrix
			Bx_offset = int(self.cachedBH[0][2]); By_offset = int(self.cachedBH[1][2])
			dst_img = cv2.warpPerspective(src=imageA, M=self.cachedAH, 
				dsize=(self.ABSize[0], self.ABSize[1]))
			dst_img[Bx_offset:Bx_offset+imageB.shape[0], 
					By_offset:By_offset+imageB.shape[1]] = imageB
			# self.draw_descriptors(img_src=dst_img)
			return dst_img
		else: # Otherwise return images concatenated
			return np.concatenate(images, axis=1)
	
	def reset(self):		
		self.cachedBH = None # Rotation/translation inverse matrix of image B
		self.cachedBINVH = None # Rotation/translation inverse matrix of image B
		self.Bpts= None # Conners of image B in stitcher result
		self.cachedAH = None # Rotation/translation inverse matrix of image A
		self.cachedAINVH = None # Rotation/translation inverse matrix of image A
		self.Apts= None # Conners of image A in stitcher result
		self.matches = None # Matches point between image A and B
		self.status = None # Status of matches between image A and B
		self.ABSize=None # Size of stitcher result

	def calibrate(self, images, ratio=0.75, reprojThresh=4.0):
		""" Finds a transposed correlation between images with key points 
			correlated
		Args:
			images: `list` of cv2.math with images to calibrate stitcher
		Returns:
		"""

		# Reset stitcher
		self.reset()

		# Unpack images
		(imageB, imageA) = images

		# detect keypoints and extract
		kpsA, featuresA = self.detectAndDescribe(imageA) 
		kpsB, featuresB = self.detectAndDescribe(imageB)
		if kpsA is None or kpsB is None: return

		# match features between the two images
		self.cachedAH, self.matches, self.status = self.matchKeypoints(kpsA=kpsA, 
			kpsB=kpsB, featuresA=featuresA, featuresB=featuresB, ratio=ratio, 
			reprojThresh=reprojThresh)

		# Calculates inverse and other components
		if self.cachedAH is not None:
			
			# Calculate translation component
			Apts=[get_projection_point_dst(pt_src=(pt[0], pt[1], 1), M=self.cachedAH) for pt in 
				[(0, 0), (imageA.shape[1], 0), (imageA.shape[1], imageA.shape[0]), (0, imageA.shape[0])]]
			
			Ax_coords=[pt[0] for pt in Apts]
			Ay_coords=[pt[1] for pt in Apts]

			Ax_min_coord = min(Ax_coords) if min(Ax_coords)<0 else 0
			Ay_min_coord = min(Ay_coords) if min(Ay_coords)<0 else 0

			self.cachedBH = np.float32([
				[1, 0, abs(Ax_min_coord)], # Translation in x axis
				[0, 1, abs(Ay_min_coord)], # Translation in y axis
				[0, 0, 1]]) # Translation in Z axis
			self.cachedBINVH= np.linalg.inv(self.cachedBH) # Inverse

			self.cachedAH[0][2]+=abs(Ax_min_coord) # Translation in x axis
			self.cachedAH[1][2]+=abs(Ay_min_coord) # Translation in y axis
			self.cachedAINVH= np.linalg.inv(self.cachedAH) # Inverse

			# Re-calculate points  of image A in stitchers result
			self.Bpts=[get_projection_point_dst(pt_src=(pt[0], pt[1], 1), M=self.cachedBH) for pt in 
				[(0, 0), (imageB.shape[1], 0), (imageB.shape[1], imageB.shape[0]), (0, imageB.shape[0])]]

			# Re-calculate points  of image B in stitchers result
			self.Apts=[get_projection_point_dst(pt_src=(pt[0], pt[1], 1), M=self.cachedAH) for pt in 
				[(0, 0), (imageA.shape[1], 0), (imageA.shape[1], imageA.shape[0]), (0, imageA.shape[0])]]

			x_coords=[pt[0] for pt in np.concatenate((self.Apts, self.Bpts), axis=1)]
			y_coords=[pt[1] for pt in np.concatenate((self.Apts, self.Bpts), axis=1)]

			self.ABSize=(
				int(abs(max(x_coords)-min(x_coords))), # Stitcher result width
				int(abs(max(y_coords)-min(y_coords)))) # Stitcher result height

		else:
			self.reset()

	def detectAndDescribe(self, image):
		""" Find key-points in image
		Args:
			image: `cv2.math` input image to find descriptors and features
		Returns:
			kps: `numpy.ndarray` collection of key-points. Key-points for which a
				  descriptor cannot be computed are removed. Sometimes new 
				  key-points can be added, for example: SIFT duplicates key-point 
				  with several dominant orientations (for each orientation).
			features: `numpy.ndarray`  Computed descriptors. In the second 
					   variant of the method descriptors[i] are descriptors 
					   computed for a key-points[i]. Row j is the key-points 
					   (or key-points[i]) is the descriptor for key-point j-th 
					   key-point.
		"""

		# convert the image to gray scale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# check to see if we are using OpenCV 3.X
		if is_cv3(or_better=False):
			# detect and extract features from the image
			try:
				descriptor = cv2.xfeatures2d.SIFT_create()
			except:
				self.debugger(DEBUG_LEVEL_0, "OpenCV is not a contrib version, check for the module xfeatures2d", 
					log_type="err")
				return None, None

			kps, features = descriptor.detectAndCompute(image, None)

		# otherwise, we are using OpenCV 2.4.X
		else:

			# detect key-points in the image
			detector = cv2.FeatureDetector_create("SIFT")
			kps = detector.detect(gray)

			# extract features from the image
			extractor = cv2.DescriptorExtractor_create("SIFT")
			kps, features = extractor.compute(gray, kps)

		# convert the key-points from Key-point objects to NumPy
		# arrays
		kps = np.float32([kp.pt for kp in kps])

		# return a tuple of key-points and features
		return kps, features

	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB, ratio=0.75, 
		reprojThresh=4.0):
		""" Match correlated points between lists featuresA and featuresB
		Args:
			kpsA: `numpy.ndarray` Coordinates of the points in the original plane
			kpsB: `numpy.ndarray` Coordinates of the points in the target plane
			featuresA: `numpy.ndarray` list of features in original plane
			featuresB: `numpy.ndarray` list of features in target plane
			ratio: 'float' aspect ratio for panoramic result (Keep it in 0.75)
			reprojThresh: 'float' correlation value for desired matched points 
		Returns:
			matches: `list` description
			H: `numpy.ndarray` perspective transformation between the source 
							   and the destination planes
			status: `numpy.ndarray` list of correlation state for each feature paired
		"""

		# compute the raw matches and initialize the list of actual matches
		matcher = cv2.DescriptorMatcher_create("BruteForce")
		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
		matches = []; H=None; status=None

		# loop over the raw matches
		for m in rawMatches:

			# ensure the distance is within a certain ratio of each
			# other (i.e. Lowe's ratio test)
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				matches.append((m[0].trainIdx, m[0].queryIdx))

		# computing a homography requires at least 4 matches
		if len(matches) > 4:

			# construct the two sets of points
			ptsA = np.float32([kpsA[i] for (_, i) in matches])
			ptsB = np.float32([kpsB[i] for (i, _) in matches])

			# compute the homography between the two sets of points
			H, status = cv2.findHomography(srcPoints=ptsA, dstPoints=ptsB, 
				method=cv2.RANSAC, ransacReprojThreshold=reprojThresh)

		# return the matches along with the homography matrix
		# and status of each matched point
		return H, matches, status

	def draw_descriptors(self, img_src):
		
		if self.Bpts is not None:
			for pt in self.Bpts:
				cv2.circle(img_src, tuple(pt), 2, (0, 0, 255), -1)
				cv2.circle(img_src, tuple(pt), 3, (0, 255, 255), 1)
		if self.Apts is not None:
			for pt in self.Apts:
				cv2.circle(img_src, tuple(pt), 2, (0, 0, 255), -1)
				cv2.circle(img_src, tuple(pt), 3, (255, 255, 0), 1)

 	def __str__(self):
		return "Stitcher:{}| matches:{}".format(
			self.sid, len(self.matches) if self.matches is not None else 0)

# =============================================================================