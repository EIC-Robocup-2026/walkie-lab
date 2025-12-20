#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time

ROBOT_CONFIG = MoveItConfigsBuilder(robot_name="my_robot", package_name="my_robot_moveit_config")\
                                    .robot_description_semantic("config/my_robot.srdf", {"name": "my_robot"})\
                                    .to_dict()
ROBOT_CONFIG = { 
    **ROBOT_CONFIG,
    "planning_scene_monitor": {
        "name": "planning_scene_monitor",
        "robot_description": "robot_description",
        "joint_state_topic": "/joint_states",
        "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
        "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
        "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
        "wait_for_initial_state_timeout": 10.0,
    },
    "planning_pipelines": {
        "pipeline_names": ["ompl"]
    },
    "plan_request_params": {
        "planning_attempts": 1,
        "planning_pipeline": "ompl",
        "max_velocity_scaling_factor": 1.0,
        "max_acceleration_scaling_factor": 1.0
    },
    "ompl": {
        "planning_plugins": ["ompl_interface/OMPLPlanner"],
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision"
        ],
        "response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath"
        ],
        "start_state_max_bounds_error": 0.1
    }
}

def main(args=None):
    rclpy.init(args=args)
    
    # Create a simple node for spinning
    node = Node("moveit_controller")
    
    robot = MoveItPy(node_name="moveit_py", config_dict=ROBOT_CONFIG)
    arm = robot.get_planning_component("arm")
    
    time.sleep(1.0)
    
    #------------------------JOINTS---------------------
    robot_state = RobotState(robot.get_robot_model())
    joint_values = {
        "joint1": 0.5,
        "joint2": 1.5,
        "joint3": 2.5,
        "joint4": 0.5,
        "joint5": 0.5,
        "joint6": 0.8
    }
    robot_state.joint_positions = joint_values
    
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=robot_state)
    
    plan_result = arm.plan()
    
    if plan_result:
        print("Executing...")
        robot.execute(plan_result.trajectory, controllers=[])
        
        # Spin the separate node
        rclpy.spin_once(node, timeout_sec=0.5)
        
        print("Execution complete")
    else:
        print("Planning failed")
    
    # Cleanup
    del arm
    del robot
    node.destroy_node()
    time.sleep(0.1)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()