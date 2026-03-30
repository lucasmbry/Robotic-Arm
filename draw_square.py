#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import copy
from geometry_msgs.msg import Pose

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder

# Defined names
planning_group = "arm"
end_effector = "sixth_link_1"
move_amount = 0.10  # 10 cm

def main(args=None):
    rclpy.init(args=args)

    # Move it params
    moveit_config = MoveItConfigsBuilder(
        robot_name="six_dof",
        package_name="six_dof_moveit"
    ).to_dict()

    # Initialises moveitpy and connects to the rviz instance
    arm = MoveItPy(node_name="draw_square", config_dict=moveit_config)

    # Get the planning group (named "arm")
    manipulator = arm.get_planning_component(planning_group)

    # Move to home pose
    manipulator.set_start_state_to_current_state()
    manipulator.set_goal_state(configuration_name="home")
    plan_result = manipulator.plan()
    if plan_result:
        arm.execute(plan_result.trajectory, controllers=[])

    # Get current pose
    robot_state = arm.get_robot_model().create_robot_state()
    robot_state.update()
    current_pose = robot_state.get_global_link_transform(end_effector)

    # Hold four corners in an array
    points = []

    # Define starting coordinates of end effector
    wpose = Pose()
    wpose.position.x = current_pose.translation[0]
    wpose.position.y = current_pose.translation[1]
    wpose.position.z = current_pose.translation[2]

    # Keep gripper same orientation
    wpose.orientation.w = 1.0

    # Move
    wpose.position.x += move_amount  
    points.append(copy.deepcopy(wpose))

    wpose.position.y += move_amount  
    points.append(copy.deepcopy(wpose))

    wpose.position.x -= move_amount  
    points.append(copy.deepcopy(wpose))

    wpose.position.y -= move_amount  
    points.append(copy.deepcopy(wpose))

    # Now plan cartesian straigh line path
    (cartesian_plan, fraction) = manipulator.compute_cartesian_path(points, 0.01, 0.0)

    # Fraction tells how successful the path planning was
    if fraction == 1.0:
        arm.execute(cartesian_plan, controllers=[])
    else:
        print("Fraction: " + str(fraction))

    rclpy.shutdown()

if __name__ == "__main__":
    main()