from sqlalchemy.orm import sessionmaker
from sqlalchemy import create_engine
from lfd_storage.demonstration_storage import Base, RobotDB

DATABASE_URL = "postgresql://postgres:postgres@localhost/robot_demos"
engine = create_engine(DATABASE_URL)
Session = sessionmaker(bind=engine)
session = Session()

def seed_data():
    robots_data = [
        RobotDB(name="fr3", config={"robot_ns": "fr3", "planning_group": "fr3_arm", "base_frame": "fr3_link0", "ee_frame": "fr3_hand_tcp"}),
        RobotDB(name="yumi_l", config={"robot_ns": "yumi_l", "planning_group": "left_arm", "base_frame": "yumi_base_link", "ee_frame": "gripper_l_tip"}),
        RobotDB(name="yumi_r", config={"robot_ns": "yumi_r", "planning_group": "right_arm", "base_frame": "yumi_base_link", "ee_frame": "gripper_r_tip"})
    ]
    session.add_all(robots_data)
    session.commit()

if __name__ == "__main__":
    seed_data()
    session.close()
