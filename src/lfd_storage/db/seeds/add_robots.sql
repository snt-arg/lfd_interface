-- Seed data for the robots table
INSERT INTO robots (name, config) VALUES
    ('fr3', '{"robot_ns": "fr3", "planning_group": "fr3_arm", "base_frame": "fr3_link0", "ee_frame": "fr3_hand_tcp"}'),
    ('yumi_l', '{"robot_ns": "yumi_l", "planning_group": "left_arm", "base_frame": "yumi_base_link", "ee_frame": "gripper_l_tip"}'),
    ('yumi_r', '{"robot_ns": "yumi_r", "planning_group": "right_arm", "base_frame": "yumi_base_link", "ee_frame": "gripper_r_tip"}');
