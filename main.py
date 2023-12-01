import general_robotics_toolbox as grt
from Arm_Lib import *
import numpy as np

# GLOBALS
NUM_JOINTS = 6
MAX_ANGLES_D = [180,180,180,180,270,180] # in deg
MIN_ANGLES_D = [0,0,0,0,0,0]
MAX_ANGLES_R = np.array([1.,1.,1.,1.,1.5,1.])*np.pi
JOINT_VELOCITY_LIMITS = np.array([20,20,20,20,20,20])*(np.pi/180)
JOINT_ACCELERATION_LIMITS = np.array([5,5,5,5,5,5])*(np.pi/180)


def deg2rad(d):
    return (d*np.pi)/180
def rad2deg(r):
    return (r*180)/np.pi


def main(Arm):
    # Construct Robot object from 'grt'
    x = np.array([1,0,0])
    y = np.array([0,1,0])
    z = np.array([0,0,1])

    # 3 x N matrix containing direction of joint as unit vectors
    H = np.array([z,y,y,y,z,y]).T
    P = None # TODO
    # Joint type
    JT = [0,0,0,0,0,0]
    # Joint Lower Limit
    JLL = MIN_ANGLES
    # Joint Upper Limit
    JUL = MAX_ANGLES_R.T
    # Joint velocity Limit
    JVL = JOIN_VELOCITY_LIMITS
    # Joint acceleration Limit
    JAL = JOINT_ACCELERATION_LIMITS
    # Spacial inertial matrices for links (Optional)
    M = None
    # Rotation matrix for tool frame (Optional)
    R_TOOL = None
    # 3x1 vector for tool frame (Optional)
    P_TOOL = None
    # Joint names (Optional)
    JNS = None
    # Robot link names (Optional) (URDF)
    RLN = None
    # Tip Link Name (Optional) (URDF)
    TLN = None
    # T_flange: optional transform between end of kinematic chain and tool frame.
    TF = None
    # T_Base transform, optional transform of base of robot in world frame.
    TBT = None

    MEFABOT = grt.Robot()


if __name__ == "__main__":
    Arm = Arm_Device()
    main(Arm)