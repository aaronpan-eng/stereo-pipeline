import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml

# opencv function to draw epipolar lines
def drawlines(img1src, img2src, lines, pts1src, pts2src):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r, c = img1src.shape[:2]
    if img1src is not None and img1src.shape[2] == 3:
        img1color = img1src
        img2color = img2src
    else:
        img1color = cv2.cvtColor(img1src, cv2.COLOR_GRAY2BGR)
        img2color = cv2.cvtColor(img2src, cv2.COLOR_GRAY2BGR)
    np.random.seed(0)
    for r, pt1, pt2 in zip(lines, pts1src, pts2src):
        color = tuple(np.random.randint(0, 255, 3).tolist())
        x0, y0 = map(int, [0, -r[2]/r[1]])
        x1, y1 = map(int, [c, -(r[2]+r[0]*c)/r[1]])
        img1color = cv2.line(img1color, (x0, y0), (x1, y1), color, 1)
        img1color = cv2.circle(img1color, tuple(map(int,pt1)), 5, color, -1)
        img2color = cv2.circle(img2color, tuple(map(int,pt2)), 5, color, -1)
    return img1color, img2color

# grab current directory
cwd = os.getcwd()

# grab images and their directories; starting with one image pair for now
im_dir = os.path.join("..", "..", "Downloads","MH_01_easy","mav0")

cam_list = ["cam0", "cam1"]
K_list = []
dist_list = []
for cam in cam_list:
    calib_yaml_path = os.path.join("..", "..", "Downloads", "MH_01_easy", "mav0", cam, "sensor.yaml")
    with open(calib_yaml_path, 'r') as f:
        calib_data = yaml.safe_load(f)
    intrinsics = calib_data["intrinsics"]
    fx, fy, cx, cy = intrinsics
    K_list.append(np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]))
    distortion_coefficients = np.array(calib_data["distortion_coefficients"])
    dist_list.append(distortion_coefficients)

    if cam == "cam0":
        left_im = os.path.join(im_dir, cam, "data", "1403636579763555584.png")
    else:
        right_im = os.path.join(im_dir,  cam, "data", "1403636579763555584.png")

# grab calibration and distortion coefficients
K1,K2 = K_list
dist1,dist2 = dist_list

# reading left and right greyscale images
left = cv2.imread(left_im)
right = cv2.imread(right_im)

# IMPORTANT: Undistort images first before feature detection
# This ensures keypoints follow the pinhole camera model
left_undist = cv2.undistort(left, K1, dist1)
right_undist = cv2.undistort(right, K2, dist2)

# using orb for kpts and descriptors on UNDISTORTED images
orb = cv2.ORB_create()
kpts1, desc1 = orb.detectAndCompute(left_undist, None)
kpts2, desc2 = orb.detectAndCompute(right_undist, None)

# using brute force matching
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
matches_knn = bf.knnMatch(desc1, desc2, k=2)
matches = []
for match1, match2 in matches_knn:
    if match1.distance < 0.7 * match2.distance:
        matches.append(match1)

# extracting point correspondneces
pts1 = np.float32([kpts1[m.queryIdx].pt for m in matches])
pts2 = np.float32([kpts2[m.trainIdx].pt for m in matches])

h1,w1 = left_undist.shape[:2]
h2,w2 = right_undist.shape[:2]

# calculate essential matrix and recover pose
E, mask = cv2.findEssentialMat(pts1, pts2, K1, cv2.RANSAC, 0.999, 1.0)

# keep inliers
pts1 = pts1[mask.ravel()==1] 
pts2 = pts2[mask.ravel()==1]
matches_inliers = [m for m, keep in zip(matches, mask.ravel()) if keep]

# recover pose and stereo rectify
_,R,t,mask_pose = cv2.recoverPose(E, pts1, pts2, K1)
R1,R2,P1,P2,Q,roi1,roi2 = cv2.stereoRectify(K1, dist1, K2, dist2, (w1, h1),R,t,alpha=0.0)

# show matches (using undistorted images)
image_matches = cv2.drawMatches(left_undist, kpts1, right_undist, kpts2, matches_inliers[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.figure(figsize=(14, 8))  # larger figure
plt.imshow(cv2.cvtColor(image_matches, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()

# compute fundamental matrix from essential matrix
F = np.linalg.inv(K2.T) @ E @ np.linalg.inv(K1)
 
# add epilines (use undistorted images and match point counts)
# Select first 10 points for visualization
pts1_viz = pts1[:10]
pts2_viz = pts2[:10]

# Compute epilines for points in image 2, draw on image 1
lines1 = cv2.computeCorrespondEpilines(pts2_viz.reshape(-1,1,2), 2, F)
lines1 = lines1.reshape(-1,3)
img3,img4 = drawlines(left_undist, right_undist, lines1, pts1_viz, pts2_viz)

# Compute epilines for points in image 1, draw on image 2
lines2 = cv2.computeCorrespondEpilines(pts1_viz.reshape(-1,1,2), 1, F)
lines2 = lines2.reshape(-1,3)
img5,img6 = drawlines(right_undist, left_undist, lines2, pts2_viz, pts1_viz)
plt.figure(figsize=(14, 8))
plt.subplot(121), plt.imshow(cv2.cvtColor(img3, cv2.COLOR_BGR2RGB))
plt.subplot(122), plt.imshow(cv2.cvtColor(img5, cv2.COLOR_BGR2RGB))
plt.suptitle("Epilines in both images")
plt.show()

# rectify images using the rectification transforms from stereoRectify
map1x, map1y = cv2.initUndistortRectifyMap(K1, dist1, R1, P1, (w1, h1), cv2.CV_32FC1)
map2x, map2y = cv2.initUndistortRectifyMap(K2, dist2, R2, P2, (w2, h2), cv2.CV_32FC1)
rectL = cv2.remap(left, map1x, map1y, cv2.INTER_LINEAR)
rectR = cv2.remap(right, map2x, map2y, cv2.INTER_LINEAR)

plt.figure(figsize=(16, 6))
plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(rectL, cv2.COLOR_BGR2RGB))
plt.title("Rectified Left")
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(rectR, cv2.COLOR_BGR2RGB))
plt.title("Rectified Right")
plt.axis('off')
plt.show()

# Optionally draw horizontal lines to verify rectification
plt.figure(figsize=(16, 6))
rectL_lines = rectL.copy()
rectR_lines = rectR.copy()
for i in range(0, h1, 50):  # draw line every 50 pixels
    cv2.line(rectL_lines, (0, i), (w1, i), (0, 255, 0), 1)
    cv2.line(rectR_lines, (0, i), (w2, i), (0, 255, 0), 1)
    
plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(rectL_lines, cv2.COLOR_BGR2RGB))
plt.title("Rectified Left with Horizontal Lines")
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(rectR_lines, cv2.COLOR_BGR2RGB))
plt.title("Rectified Right with Horizontal Lines")
plt.axis('off')
plt.show()