import math
from robodk import *
from robodk.robolink import TargetReachError

RDK = robolink.Robolink()

INSTRUCTION_INSERT_CODE = 0

# Global speed and acceleration settings
GLOBAL_SPEED = 600           # Default speed in mm/s
GLOBAL_ACCELERATION = 500     # Default acceleration in mm/s^2

# Tray slot positions (XY in mm)
tray_coords = {
    0: [-414.4, -456.17],    # Endcap (robot 2: [-483.39, -462.03])
    1: [-334.85, -456.72],  # Battery (robot 2: [-402.396, -459.981])
    2: [-259.08, -457.45],  # Head (robot 2: [-326.785, -459.887])
    3: [-182.53, -456.90],     # Pedestal (robot 2: [-251.57, -460.99])
    4: [-256.8, -458.1]   # Release finished flashlight into tray
}

CLEAR_HEIGHT = 130            # Approach (clear) height (mm)
HOME_POSITION = [0, -90, 0, -90, 0, 0]        # Home position in joint degrees
TEST_POSITION = [90, -90, 90, 270, -90, -270]   # Closer-to-table position in joint degrees

# Grip heights for each component (in mm)
GRIP_HEIGHTS = {
    "endcap": 10, # Robot 2:
    "battery": 34, # Robot 2: 24.708
    "head": 65.3, # Robot 2: 52.95
    "pedestal": 60.9 # Robot 2:
}

# Clamp positions
CLAMP_XY = [-322.98, 38.3] # Robot 2: [-330.53, 34.77]
CLAMP_HEIGHTS = {
    "release_head": 163.9, # Robot 2: 151.33
    "release_battery": 192.751,
    "release_endcap": 197.31, # Robot 2: 189
    "pickup_flashlight": 187,
    "clear": 250
}

# Joint targets for endcap-to-pedestal motion (all values in degrees)
ENDCAP_CLR_JOINTS   = [37.41, -72.76, 83.21, -100.44, -89.73, -142.51]           # Robot 2: [34.17, -65.64, 73.24, -97.56, -89.73, -145.75]
INTERMEDIATE_JOINTS = [8.348806, -77.787199, 103.336014, -25.548815, 8.348806, -194.849375]
PED_CLR_JOINTS      = [15.32, -96.4, 143.41, -47.02, 15.3, -180]             # Robot 2: [12.62, -87.67, 134.47, -46.81, 12.61, -179.99]
PED_DROP_JOINTS     = [15.31, -84.25, 147.34, -63.09, 15.3, -180]          # Robot 2: [12.61, -78.15, 137.70, -59.41, 12.65, -180.15]

# Global clamp parameters
GRIP_WAIT_TIME = 2.0           # Seconds to wait after gripper operations
CLAMP_APPROACH_SPEED = 200     # Speed in mm/s when approaching the clamp
CLAMP_OPERATION_SPEED = 100    # Speed in mm/s during clamp operation

# ------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------
def set_motion_params(speed, acceleration, robot):
    """Sets the robot's speed and acceleration to the global settings."""
    robot.setSpeed(speed)
    robot.setAcceleration(acceleration)

def tray_pose(tray_index, z):
    """
    Returns a standard Cartesian Pose for a given tray slot and Z height,
    using orientation [180, 0, 90] (in degrees).
    """
    x, y = tray_coords[tray_index]
    return Pose(x, y, z, 180, 0, 90)

# ------------------------------------------------------------------
# Core Functions
# ------------------------------------------------------------------
def goto_and_pickup_tray(tray_slot_index, pickup_height, clear_height, robot):
    """
    Moves the robot to a tray slot:
      - Approaches at the clear height.
      - Lowers to the pickup height.
      - Closes the gripper.
      - Raises back to the clear height.
    
    For tray slot 0 (endcap), uses the preferred joint target.
    """
    set_motion_params(1000, GLOBAL_ACCELERATION, robot)
    
    if tray_slot_index == 0:
        robot.MoveJ(ENDCAP_CLR_JOINTS)
        robot.MoveL(Pose(tray_coords[0][0], tray_coords[0][1], pickup_height, 180, 0, 90))
        robot.RunCodeCustom('rq_close_and_wait()', INSTRUCTION_INSERT_CODE)
        robot.MoveL(Pose(tray_coords[0][0], tray_coords[0][1], clear_height, 180, 0, 90))
    else:
        approach = tray_pose(tray_slot_index, clear_height)
        pickup   = tray_pose(tray_slot_index, pickup_height)
        robot.MoveJ(approach)
        robot.MoveL(pickup)
        robot.RunCodeCustom('rq_close_and_wait()', INSTRUCTION_INSERT_CODE)
        robot.MoveL(approach)

def goto_and_release_tray(tray_slot_index, release_height, clear_height, robot):
    """
    Moves the robot to a tray slot:
      - Approaches at the clear height.
      - Lowers to the release height.
      - Opens the gripper.
      - Raises back to the clear height.
    """
    approach = tray_pose(tray_slot_index, clear_height)
    release  = tray_pose(tray_slot_index, release_height)
    set_motion_params(1000, GLOBAL_ACCELERATION, robot)
    robot.MoveL(approach)
    robot.setSpeed(200)
    robot.MoveL(release)
    robot.RunCodeCustom('rq_open()', INSTRUCTION_INSERT_CODE)
    robot.MoveL(approach)

def move_endcap_to_pedestal(robot):
    """
    Moves the endcap (already grasped at Endcap CLR) to the pedestal by:
      1. Moving via an intermediate joint target.
      2. Moving to the Pedestal clear height (joint move for orientation).
      3. Lowering linearly (MoveL) from Pedestal clear height to the pedestal grip height.
      4. Releasing the gripper.
      5. Raising linearly back to Pedestal clear height.
    """
    set_motion_params(1000, GLOBAL_ACCELERATION, robot)
    robot.MoveJ(INTERMEDIATE_JOINTS)
    robot.MoveJ(PED_CLR_JOINTS)
    robot.MoveL(PED_DROP_JOINTS)
    robot.RunCodeCustom('rq_open()', INSTRUCTION_INSERT_CODE)
    robot.MoveJ(PED_CLR_JOINTS)

# ------------------------------------------------------------------
# Integrated Clamp Functions
# ------------------------------------------------------------------
def pickup_from_clamp(pickup_height, clear_height, robot):
    """
    Picks up an item from the clamp:
      - Approaches the clamp at clear height.
      - Lowers to pickup height.
      - Closes the gripper to grasp the item.
      - Raises back to the clear height.
    """
    set_motion_params(1000, GLOBAL_ACCELERATION, robot)
    safe_pose = Pose(CLAMP_XY[0], CLAMP_XY[1], clear_height, 180, 0, 90)
    pickup_pose = Pose(CLAMP_XY[0], CLAMP_XY[1], pickup_height, 180, 0, 90)
    robot.setSpeed(200)
    robot.MoveL(pickup_pose)
    robot.RunCodeCustom('rq_close_and_wait()', INSTRUCTION_INSERT_CODE)
    robot.MoveL(safe_pose)

def release_into_clamp(release_height, clear_height, robot):
    """
    Releases an item into the clamp:
      - Approaches the clamp at clear height.
      - Lowers to release height.
      - Opens the gripper to release the item.
      - Raises back to the clear height.
    """
    set_motion_params(1000, GLOBAL_ACCELERATION, robot)
    safe_pose = Pose(CLAMP_XY[0], CLAMP_XY[1], clear_height, 180, 0, 90)
    release_pose = Pose(CLAMP_XY[0], CLAMP_XY[1], release_height, 180, 0, 90)
    robot.MoveJ(safe_pose)
    robot.setSpeed(200)
    robot.MoveL(release_pose)
    robot.RunCodeCustom('rq_open()', INSTRUCTION_INSERT_CODE)
    robot.MoveL(safe_pose)




def tighten_cap(robot):

    set_motion_params(1000, GLOBAL_ACCELERATION, robot)
    # Define clamp coordinates and heights (from global variables)
    new_clamp_x = -322.25 # Robot 2: -330.9
    new_clamp_y = 38.78 # Robot 2: 35.86
    safe_pose = Pose(new_clamp_x, new_clamp_y, CLAMP_HEIGHTS["clear"], 180, 0, 90)
    # release_pose = Pose(new_clamp_x, new_clamp_y, CLAMP_HEIGHTS["release_endcap"], 180, 0, 90)
    
    # Base turning angles (only joint 6 changes):
    # "Initial" (0° target) and "Final" (~180° target)
    # anti_30 = [-26.509508, -115.182409, 112.727496, -87.535252, -90.000530, -249.119509] # Move anticlockwise 30 degrees while applying "0.3-mm" downward pressure
    initial_turning_angle = [-26.51, -115.18, 112.67, -87.48, -90, -206.51] # Robot 2: [-25.27, -114.01, 113.04, -89.01, -89.94, -205.07]
    final_turning_angle   = [-26.510124, -115.194661, 113.040864, -87.836005, -89.999890, 11.929876] # Robot 2: [-25.27, -114.01, 113.27, -89.23, -89.94, -31.58]
    # final_turning_angle goes down by 2.5 mm
    
    # Engage the clamp.
    robot.RunCodeCustom('clamp()', INSTRUCTION_INSERT_CODE)
    
    # Move to the safe approach pose and then lower to the release pose.
    robot.MoveJ(safe_pose)
    robot.MoveL(initial_turning_angle)

    # Turn 30 degrees anticlockwise
    # robot.MoveJ(anti_30)
    # robot.RunCodeCustom('rq_open()', INSTRUCTION_INSERT_CODE)

    robot.setSpeedJoints(700)

    # Toggle between the two turning angles for a number of cycles.
    for i in range(6):
        robot.RunCodeCustom('rq_close_and_wait()', INSTRUCTION_INSERT_CODE)
        robot.MoveJ(final_turning_angle)
        robot.RunCodeCustom('rq_open()', INSTRUCTION_INSERT_CODE)
        robot.setSpeedJoints(200)
        robot.MoveJ(initial_turning_angle)

    # Final torque-based tightening followed by unclamping.
    #tighten_torque_pose = [-26.509597, -115.227184, 114.226946, -88.989923, -90.000438, -206.509597]
    tighten_torque_pose = [-26.51, -115.18, 112.67, -87.48, -90, -206.51]
    robot.MoveJ(tighten_torque_pose)

    #tighten_torque(torqueLimit, startAngle, endAngle, jointAccel, jointSpeed,num_checkTorque, gripperForce, gripperSpeed, gripperOpen)
    robot.RunCodeCustom('tighten_torque(2, -205.27, -115.27, 2, 2, 1, 100, 100, 50)', INSTRUCTION_INSERT_CODE) #2, -90, 51.56, 2, 2, 1, 100, 100, 50
    robot.RunCodeCustom('unclamp()', INSTRUCTION_INSERT_CODE)











# ------------------------------------------------------------------
# Main Program: Assemble the Flashlight
# ------------------------------------------------------------------
def main():
    robot = RDK.Item('UR5')
    
    # Unclamp at start in case clamp is stuck
    robot.RunCodeCustom('unclamp()', INSTRUCTION_INSERT_CODE)
    
    # Step 1: Start from Home
    robot.MoveJ(HOME_POSITION)
    set_motion_params(1000, GLOBAL_ACCELERATION, robot)

    # Step 2: Move the endcap to the pedestal
    goto_and_pickup_tray(0, GRIP_HEIGHTS["endcap"], CLEAR_HEIGHT, robot)
    move_endcap_to_pedestal(robot)
    
    # Step 3: Move the flashlight head into the clamp
    goto_and_pickup_tray(2, GRIP_HEIGHTS["head"], CLEAR_HEIGHT, robot)
    release_into_clamp(CLAMP_HEIGHTS["release_head"], CLAMP_HEIGHTS["clear"], robot)
    
    # Step 4: Move the battery to the flashlight head
    goto_and_pickup_tray(1, GRIP_HEIGHTS["battery"], CLEAR_HEIGHT, robot)
    release_into_clamp(CLAMP_HEIGHTS["release_battery"], CLAMP_HEIGHTS["clear"], robot)
    
    # Step 5: Move the endcap from the pedestal to the clamp, then tighten it.
    goto_and_pickup_tray(3, GRIP_HEIGHTS["pedestal"], CLEAR_HEIGHT, robot)
    tighten_cap(robot)
    
    # Step 6: After tightening, move the assembled flashlight back to tray slot 2 (flashlight head)
    pickup_from_clamp(185, CLAMP_HEIGHTS["clear"], robot)
    goto_and_release_tray(4, 89.90, CLEAR_HEIGHT, robot)
    
    # End: Return to Home
    robot.MoveJ(HOME_POSITION)

if __name__ == "__main__":
    main()