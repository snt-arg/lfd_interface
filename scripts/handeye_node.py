#!/usr/bin/env python3

import os
import yaml
import rospy
import curses

from lfd_camera.handeye import DataCollection, Solver


# Define a custom dumper
class MyDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, False)

def my_represent_list(dumper, data):
    if isinstance(data, list) and all(isinstance(i, list) for i in data):
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=None)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)


def update_config(path, robot, camera):
    config = {
        'robot': robot,
        'camera': camera
    }
    with open(path, 'w') as file:
        yaml.dump(config, file, Dumper=MyDumper, default_flow_style=None)


def main(stdscr):
    rospy.init_node('handeye_node', anonymous=True)

    # Add the custom representer to the dumper
    MyDumper.add_representer(list, my_represent_list)

    config_path = rospy.get_param("~config_path")
    os.makedirs(os.path.dirname(config_path), exist_ok=True)

    data_collection = DataCollection()

    robot = []
    camera = []

    # Read existing records from config file
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
        if 'robot' in config:
            robot = config['robot']
        if 'camera' in config:
            camera = config['camera']

    stdscr.addstr("Press R to Record Robot Pose, Press S to Solve the Calibration\n")
    while not rospy.is_shutdown():

        key = stdscr.getch()

        if key == ord('r'):
            robot_pose = data_collection.read_robot_pose()
            robot.append(robot_pose)
            stdscr.addstr("Robot Pose: {}\n".format(robot_pose))
            stdscr.addstr("Press C to Record Camera Pose\n")

        elif key == ord('c'):
            camera_pose = data_collection.read_camera_pose()
            camera.append(camera_pose)
            stdscr.addstr("Camera Pose: {}\n".format(camera_pose))
            update_config(config_path, robot, camera)
            stdscr.addstr("---------------\n")
            stdscr.addstr("New Record Added\n")
            stdscr.addstr("---------------\n")
            stdscr.addstr("Press R to Record Robot Pose, Press S to Solve the Calibration\n")
        
        elif key == ord('s'):
            stdscr.addstr("---------------\n")
            stdscr.addstr("Solving the Calibration\n")
            stdscr.addstr("---------------\n")
            solver = Solver(config_path)
            result, tf = solver.solve()
            stdscr.addstr("Success? {}\n".format(result.is_success()))
            stdscr.addstr("Transformation Matrix =\n{}\n".format(tf))
            stdscr.addstr("optimal cost = {}\n".format(result.get_optimal_cost()))
            stdscr.addstr("solver is: {}\n".format(result.get_solver_id().name()))
            stdscr.addstr("---------------\n")
            stdscr.addstr("Calibration Solved\n")
            stdscr.addstr("---------------\n")

        stdscr.refresh()


if __name__ == '__main__':
    curses.wrapper(main)