import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

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
im_dir = os.path.join("..", "..", "Downloads", "test_rect")
left_im = os.path.join(im_dir, "left1.png")
right_im = os.path.join(im_dir, "right1.png")

# reading left and right greyscale images
left = cv2.imread(left_im)
right = cv2.imread(right_im)

sift = cv2.SIFT_create()

kpts1, desc1 = sift.detectAndCompute(left, None)
kpts2, desc2 = sift.detectAndCompute(right, None)

# FLANN parameters
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)
 
flann = cv2.FlannBasedMatcher(index_params,search_params)
matches = flann.knnMatch(desc1,desc2,k=2)
 
pts1 = []
pts2 = []
 
# ratio test as per Lowe's paper
good_matches = []
for i,(m,n) in enumerate(matches):
    if m.distance < 0.8*n.distance:
        pts2.append(kpts2[m.trainIdx].pt)
        pts1.append(kpts1[m.queryIdx].pt)
        good_matches.append(m)

pts1 = np.int32(pts1)
pts2 = np.int32(pts2)

# calculate fundamental matrix with opencv
F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC)

# keep inliers
pts1 = pts1[mask.ravel()==1] 
pts2 = pts2[mask.ravel()==1]
matches_inliers = [m for m, keep in zip(good_matches, mask.ravel()) if keep]

# show matches
image_matches = cv2.drawMatches(left, kpts1, right, kpts2, matches_inliers[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.figure(figsize=(14, 8))  # larger figure
plt.imshow(cv2.cvtColor(image_matches, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()

h1,w1 = left.shape[:2]
h2,w2 = right.shape[:2]

# rectify using opencv and fundamental matrix
_,H1,H2 = cv2.stereoRectifyUncalibrated(np.float32(pts1), np.float32(pts2), F, imgSize=(w1,h1))

# add epilines
lines1 = cv2.computeCorrespondEpilines(pts2.reshape(-1,1,2), 2, F)
lines1 = lines1.reshape(-1,3)
img3,img4 = drawlines(left, right, lines1, pts1, pts2)

lines2 = cv2.computeCorrespondEpilines(pts1.reshape(-1,1,2), 1, F)
lines2 = lines2.reshape(-1,3)
img5,img6 = drawlines(right, left, lines2, pts2, pts1)
plt.figure(figsize=(14, 8))
plt.subplot(121), plt.imshow(cv2.cvtColor(img3, cv2.COLOR_BGR2RGB))
plt.subplot(122), plt.imshow(cv2.cvtColor(img5, cv2.COLOR_BGR2RGB))
plt.suptitle("Epilines in both images")
plt.show()

# rectify images
rectL = cv2.warpPerspective(left, H1.astype(np.float32), (w1,h1))
rectR = cv2.warpPerspective(right, H2.astype(np.float32), (w2,h2))
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