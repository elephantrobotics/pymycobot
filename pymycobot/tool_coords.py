# coding=utf-8
import numpy as np


def CvtRotationMatrixToEulerAngle(pdtRotationMatrix):
    pdtEulerAngle = np.zeros(3)

    pdtEulerAngle[2] = np.arctan2(pdtRotationMatrix[1, 0], pdtRotationMatrix[0, 0])

    fCosRoll = np.cos(pdtEulerAngle[2])
    fSinRoll = np.sin(pdtEulerAngle[2])

    pdtEulerAngle[1] = np.arctan2(-pdtRotationMatrix[2, 0], (fCosRoll * pdtRotationMatrix[0, 0]) + (fSinRoll * pdtRotationMatrix[1, 0]))
    pdtEulerAngle[0] = np.arctan2((fSinRoll * pdtRotationMatrix[0, 2]) - (fCosRoll * pdtRotationMatrix[1, 2]), (-fSinRoll * pdtRotationMatrix[0, 1]) + (fCosRoll * pdtRotationMatrix[1, 1]))

    return pdtEulerAngle

def CvtEulerAngleToRotationMatrix(ptrEulerAngle):
    ptrSinAngle = np.sin(ptrEulerAngle)
    ptrCosAngle = np.cos(ptrEulerAngle)

    ptrRotationMatrix = np.zeros((3, 3))
    ptrRotationMatrix[0, 0] = ptrCosAngle[2] * ptrCosAngle[1]
    ptrRotationMatrix[0, 1] = ptrCosAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] - ptrSinAngle[2] * ptrCosAngle[0]
    ptrRotationMatrix[0, 2] = ptrCosAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] + ptrSinAngle[2] * ptrSinAngle[0]
    ptrRotationMatrix[1, 0] = ptrSinAngle[2] * ptrCosAngle[1]
    ptrRotationMatrix[1, 1] = ptrSinAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] + ptrCosAngle[2] * ptrCosAngle[0]
    ptrRotationMatrix[1, 2] = ptrSinAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] - ptrCosAngle[2] * ptrSinAngle[0]
    ptrRotationMatrix[2, 0] = -ptrSinAngle[1]
    ptrRotationMatrix[2, 1] = ptrCosAngle[1] * ptrSinAngle[0]
    ptrRotationMatrix[2, 2] = ptrCosAngle[1] * ptrCosAngle[0]

    return ptrRotationMatrix

def transformation_matrix_from_parameters(rotation_matrix, translation_vector):
    """
    Create a 4x4 homogeneous transformation matrix.

    :param rotation_matrix: 3x3 numpy array representing rotation.
    :param translation_vector: 1x3 numpy array representing translation.
    :return: 4x4 numpy array representing the transformation matrix.
    """
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation_vector
    return T

def get_flange_pose(flange_matrix, tool_pose):
    """
    Convert the pose from flange coordinate system to tool coordinate system.

    :param flange_to_tool_matrix: 4x4 numpy array, transformation matrix from flange to tool.
    :param flange_pose: 4x4 numpy array, pose in flange coordinate system.
    :return: 4x4 numpy array, pose in tool coordinate system.
    """
    return np.dot(flange_matrix, np.linalg.inv(tool_pose))

def get_tool_pose(flange_matrix, tool_pose):
    """
    Convert the pose from tool coordinate system to flange coordinate system.

    :param flange_to_tool_matrix: 4x4 numpy array, transformation matrix from flange to tool.
    :param tool_pose: 4x4 numpy array, pose in tool coordinate system.
    :return: 4x4 numpy array, pose in flange coordinate system.
    """
    return np.dot(flange_matrix, tool_pose)


def flangeToTool(current_coords, tool_matrix):
    # Example pose in flange coordinate system
    flange_pose = np.eye(4)
    flange_pose[:3, :3] = CvtEulerAngleToRotationMatrix(current_coords[3:6] * np.pi/180.0)
    flange_pose[:3, 3] = current_coords[:3]  # Example translation

    # Switch to tool coordinate system
    tool_pose = get_tool_pose(flange_pose, tool_matrix)
    tool_coords = np.concatenate((tool_pose[:3,3].T, CvtRotationMatrixToEulerAngle(tool_pose[:3,:3]) * 180/np.pi))

    return tool_coords

def toolToflange(tool_coords, tool_matrix):

    # Example pose in tool coordinate system
    tool_pose = np.eye(4)
    tool_pose[:3, :3] = CvtEulerAngleToRotationMatrix(tool_coords[3:6] * np.pi/180.0)
    tool_pose[:3, 3] = tool_coords[:3] # Example translation

    # Switch to tool coordinate system
    flange_pose = get_flange_pose(tool_pose, tool_matrix)
    
    flange_coords = np.concatenate((flange_pose[:3,3].T, CvtRotationMatrixToEulerAngle(flange_pose[:3,:3]) * 180/np.pi ))
    return flange_coords