import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm
from scipy.spatial.transform import Rotation
from scipy import optimize

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

def rot_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                     [np.sin(theta),  np.cos(theta), 0, 0],
                     [0,              0,             1, 0],
                     [0,              0,             0, 1]])

def rot_y(beta):
    return np.array([[np.cos(beta),   0, np.sin(beta), 0],
                     [0,              1, 0,             0],
                     [-np.sin(beta),   0,  np.cos(beta), 0],
                     [0,              0,             0, 1]])


def rot_x(alpha):
    return np.array([[1,  0,              0,             0],
                     [0,  np.cos(alpha), -np.sin(alpha), 0],
                     [0,  np.sin(alpha),  np.cos(alpha), 0],
                     [0,  0,              0,             1]])

def trans_z(d):
    return np.array([[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,d],
                     [0,0,0,1]])

def trans_x(a):
    return np.array([[1,0,0,a],
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]])

def FK_dh(joint_angles, link):
    """
    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    l0 = 0.055
    l1 = 0.0546
    l2 = 0.0546
    l3 = 0.118

    a =     np.array([0,l1,l2,0,0])
    alpha = np.array([np.pi/2,0,0,-np.pi/2,0])
    d =     np.array([l0,0,0,0,l3])
    theta = np.array([joint_angles[0],joint_angles[1] + np.pi/2,
                joint_angles[2],joint_angles[3]-np.pi/2,0])

    H = np.identity(4)

    for i in range(link):
        H = H.dot(rot_z(theta[i]).dot(trans_z(d[i])).dot(trans_x(a[i])).dot(rot_x(alpha[i])))
    return H


def FK_at(joint_angles, joint_id):
    """
    take in joint angles,
    return the pose of of joint[joint_id] in the workspace
    """
    transformation = FK_dh(joint_angles, joint_id)

    pose_xyz = get_pose_from_T(transformation)
    euler = get_euler_angles_from_T(transformation)

    return np.array([pose_xyz[0],pose_xyz[1], pose_xyz[2], euler[0], euler[1], euler[2]])

def IK(pose, verbose=False):
    # pose input is an array with x, y, z, phi, theta, psi
    j_angles = np.zeros((4, 1))

    l0 = 0.055
    l1 = 0.0546
    l2 = 0.0546
    l3 = 0.118

    R04 = Rotation.from_euler('ZYZ', pose[3:6])
    R04 = np.around(R04.as_dcm(), decimals = 5)

    o_c = pose[:3].reshape(3,1) - l0*np.array([[0],[0],[1]]) - l3*R04.dot(np.array([[0],[0],[1]]))
    print("-:")
    print(- l3*R04.dot(np.array([[0],[0],[1]])))
    print("o_c:")
    print(o_c)

    #calculate joint 0
    j_angles[0] = np.arctan2(o_c[1],o_c[0])-np.pi

    #calculate joint 1 and 2
    x1 = np.sqrt(o_c[0]**2 + o_c[1]**2)
    y1 = o_c[2]
    z = ((x1**2 + y1**2) - l1**2 - l2**2)/(2*l1*l2)
    print("z:")
    print(z)
    print(x1**2 + y1**2)
    print(l1**2 + l2**2)

    if z >= -1 and z <= 1:#reachable position
        j_angles[2] = np.arccos(z)
        #a "elbow-up" configuration is chosen here to avoid collision
        if j_angles[2] > 0:
            j_angles[2] = -j_angles[2]
        j_angles[1] = np.arctan2(y1,x1) - np.arctan2(l2*np.sin(j_angles[2]),l1+l2*np.cos(j_angles[2]))
    else:
        print("\nThe pose is not reachable")
        return np.zeros((4,1)),False

    #this is the command to the joints (theta1 and theta2)
    j_angles[1] = np.pi/2 -j_angles[1]
    j_angles[2] = -j_angles[2]

    j_angles[3] = pose[4]-j_angles[1]-j_angles[2]
    while j_angles[3] < -np.pi:
        j_angles[3] += 2*np.pi
    while j_angles[3] > np.pi:
        j_angles[3] -= 2*np.pi

    return j_angles,True

def get_euler_angles_from_T(T):
    """
    Get the euler angles of a transformation matrix.
    Args:T: 4x4 homegenous transformation matrix.
    Returns:An (n x 3) np.ndarray (roll, pitch, yaw) position.
    """
    return Rotation.from_dcm(T[:3, :3]).as_euler('ZYZ')

def get_pose_from_T(T):
    """
    Get the x, y, z pose of a transformation matrix.
    Args: T: 4x4 homegenous transformation matrix.
    Returns: An (n x 3) np.ndarray (x, y, z) position.
    """
    return T[:3,3]

def solvable_ik(cube_xyz, theta = 0, task = 0):
    """
    call this function to obtain a pair of feasible ik solutions
    for the cube(preparation and action ones).
    cube_xyz: the coordinates of the cube.
    theta: the angle of the cube (2D), 3D is not implemented.
    task: for generating different poses based on task requirement
    """
    cube = np.asarray(cube_xyz)
    temp_poses = get_grip_pose_sets(cube,theta,task)

    for i in range(int(temp_poses.shape[0]/2)):
        if (IK(temp_poses[2*i])[1] == True) and (IK(temp_poses[2*i+1])[1] == True):
            res = np.block([IK(temp_poses[2*i])[0], IK(temp_poses[2*i+1])[0]])

            #sometimes unnessary twist is obstained for joint 3 from the ZYZ decomposition
            #especially when the angle of joint 4 = 0 (redundancy).
            #round the joint 3 within joint limit
            if res[3,0] < -165*D2R or res[3,0] > 165*D2R:
                res[4,0] = -res[4,0]
                res[3,0] = 0
            if res[3,1] < -165*D2R or res[3,1] > 165*D2R:
                res[4,1] = -res[4,1]
                res[3,1] = 0

            #check whether joint 2 and 3 are within range
            if not(-120*D2R < res[2,0] < 100*D2R):
                continue
            if not(-150*D2R < res[3,0] < 150*D2R):
                continue

            #when the cube is near the round of position, the preparation pose and
            #action pose would result in ~1 cyle rotation of joint0. Avoid this.
            if np.abs(res[0,0]-res[0,1]) > np.pi*3/2:
                continue

            return(np.transpose(res))

    return np.zeros((2,6))


angs = np.array([0,0,np.pi/2,np.pi/2])
res = FK_at(angs,5)
print(res)

pos = np.array([-0.0546,0,-0.0084,0,-np.pi,0])
res = IK(pos)
print("ik res:")
print(res)
#
# test = FK_at(res[0],5)
# print("fk res:")
# print(test)
