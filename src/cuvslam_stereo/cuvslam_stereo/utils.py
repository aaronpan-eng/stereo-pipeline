import cuvslam
from scipy.spatial.transform import Rotation as R

##### Helper functions below obtained from cuvslam example here:                        #####
##### https://github.com/NVlabs/PyCuVSLAM/blob/main/examples/kitti/track_kitti_slam.py  #####
# Lambda to convert quaternion [x, y, z, w] to 3x3 rotation matrix (as list of lists)
quaternion_to_rotation_matrix = lambda q: R.from_quat(q).as_matrix().tolist()

# Lambda to multiply two quaternions [x, y, z, w] * [x, y, z, w]
quaternion_multiply = lambda q1, q2: (R.from_quat(q1) * R.from_quat(q2)).as_quat().tolist()

# Lambda to rotate a 3D vector using a 3x3 rotation matrix
rotate_vector = lambda vector, rotation_matrix: R.from_matrix(rotation_matrix).apply(vector).tolist()


def combine_poses(initial_pose, relative_pose):
    """
    Combine initial pose with relative pose to get absolute pose.
    
    Args:
        initial_pose: cuvslam.Pose object representing initial pose
        relative_pose: cuvslam.Pose object representing relative pose
    
    Returns:
        cuvslam.Pose object representing combined absolute pose
    """
    # Get rotation matrix from initial pose quaternion
    rotation_matrix = quaternion_to_rotation_matrix(initial_pose.rotation)
    
    # Rotate relative translation by initial pose rotation
    rotated_rel_t = rotate_vector(relative_pose.translation, rotation_matrix)
    
    # Add initial translation
    absolute_translation = [
        initial_pose.translation[0] + rotated_rel_t[0],
        initial_pose.translation[1] + rotated_rel_t[1],
        initial_pose.translation[2] + rotated_rel_t[2]
    ]
    
    # Multiply quaternions
    absolute_rotation = quaternion_multiply(initial_pose.rotation, relative_pose.rotation)
    
    return cuvslam.Pose(translation=absolute_translation, rotation=absolute_rotation)


def transform_landmarks(landmarks, initial_pose):
    """
    Transform landmarks by initial pose (rotation + translation).
    
    Args:
        landmarks: list of 3D landmark coordinates
        initial_pose: cuvslam.Pose object representing initial pose
    
    Returns:
        List of transformed 3D landmark coordinates
    """
    rotation_matrix = quaternion_to_rotation_matrix(initial_pose.rotation)
    transformed_landmarks = []
    
    for landmark in landmarks:
        # Rotate landmark by initial pose rotation
        rotated_landmark = rotate_vector(landmark, rotation_matrix)
        
        # Add initial translation
        transformed_landmark = [
            initial_pose.translation[0] + rotated_landmark[0],
            initial_pose.translation[1] + rotated_landmark[1],
            initial_pose.translation[2] + rotated_landmark[2]
        ]
        transformed_landmarks.append(transformed_landmark)
    
    return transformed_landmarks