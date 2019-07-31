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
	def __init__(self, images):
		
		self.Stitchers=[]
		
		pass

	def stitch(self):
		pass

	def calibrate_stitcher(self):
		pass

	def save_stitcher(self):
		pass

	def load_stitcher(self):
		pass

# =============================================================================
class StitcherBase(Debugger):
	def __init__(self):

		self.cachedH = None
		self.cachedINVH = None
		self.matches = None
		self.status = None
		
	def stitch(self, images):
		
		# unpack the images
		(imageB, imageA) = images
 
		if self.cachedH is not None:
			# apply a perspective transform to stitch the images together
			# using the cached homography matrix
			dst_img = cv2.warpPerspective(src=imageA, M=self.cachedH,
				dsize=(imageA.shape[1]+imageB.shape[1], imageA.shape[0]))
			dst_img[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
		else:
			dst_img=np.concatenate(images,axis=1)

		# return the stitched image
		return dst_img
	
	def calibrate(self, images, ratio=0.75, reprojThresh=4.0):
		""" Finds a transposed correlation between images with key points 
			correlated
		Args:
			images: `list` of cv2.math with images to calibrate stitcher
		Returns:
		"""

		# Unpack images
		(imageB, imageA) = images

		# detect keypoints and extract
		kpsA, featuresA = self.detectAndDescribe(imageA) 
		kpsB, featuresB = self.detectAndDescribe(imageB)
		if kpsA is None or kpsB is None: return

		# match features between the two images
		self.cachedH, self.matches, self.status = self.matchKeypoints(kpsA=kpsA, 
			kpsB=kpsB, featuresA=featuresA, featuresB=featuresB, ratio=ratio, 
			reprojThresh=reprojThresh)
		if self.cachedH is not None:
			self.cachedINVH= np.linalg.inv(self.cachedH)

		self.debugger(DEBUG_LEVEL_0, "[STITCHER]: Calibrated", log_type="info")

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

	def draw_match_points(self):
		pass