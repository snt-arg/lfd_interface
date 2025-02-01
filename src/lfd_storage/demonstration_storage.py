
from fileinput import filename
from logging import exception
import os
import rospy
import pickle
import re
import psycopg2
import json

from sqlalchemy import (
    create_engine, Column, Integer, Text, DateTime, ForeignKey, String, func, LargeBinary, Index
)
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship, sessionmaker

from lfd_interface.msg import DemonstrationMsg
from lfd_interface.srv import GetDemonstration, GetDemonstrationRequest, GetDemonstrationResponse, DemoCount, DemoCountResponse


Base = declarative_base()

class RobotDB(Base):
    __tablename__ = 'robots'
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(Text, nullable=False)
    config = Column(JSONB)
    created_at = Column(DateTime, server_default=func.now())
    
    # Relationship to Demonstration (optional if you want easy access from Robot to Demonstrations)
    demonstrations = relationship(
        'DemonstrationDB',
        back_populates='robot',
        cascade='all, delete-orphan'
    )
    
    # Option 1: Create index by passing 'index=True'
    # name = Column(Text, nullable=False, index=True)
    
    # Option 2: Create an explicit index
    __table_args__ = (
        Index('idx_robot_name', 'name'),
    )


class DemonstrationDB(Base):
    __tablename__ = 'demonstrations'
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(Text, nullable=False)
    robot_id = Column(Integer, ForeignKey('robots.id', ondelete='CASCADE'), nullable=False)
    meta_data = Column(JSONB)
    created_at = Column(DateTime, server_default=func.now())
    
    # Relationship to Robot
    robot = relationship('RobotDB', back_populates='demonstrations')
    
    # Relationship to Trajectory (optional for easy access)
    trajectories = relationship(
        'TrajectoryDB',
        back_populates='demonstration',
        cascade='all, delete-orphan'
    )
    
    __table_args__ = (
        Index('idx_demo_name', 'name'),
    )


class TrajectoryDB(Base):
    __tablename__ = 'trajectories'
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    demo_id = Column(Integer, ForeignKey('demonstrations.id', ondelete='CASCADE'), nullable=False)
    type = Column(String(50), nullable=False)
    trajectory_data = Column(LargeBinary, nullable=False)
    created_at = Column(DateTime, server_default=func.now())
    
    # Relationship to Demonstration
    demonstration = relationship('DemonstrationDB', back_populates='trajectories')
    
    __table_args__ = (
        Index('idx_trajectory_type', 'type'),
    )


class DemonstrationStorage(object):

    def __init__(self):
        DATABASE_URL = "postgresql://postgres:postgres@localhost/robot_demos"
        engine = create_engine(DATABASE_URL)
        Base.metadata.create_all(engine)
        Session = sessionmaker(bind=engine)
        self.session = Session()
        # self.dir_template = "demonstrations/{}"
        # self.filename_template = "demonstrations/{}/{}.pickle"
        rospy.Subscriber("save_demonstration", DemonstrationMsg, self.subcb_save_trajectory)
        self.service = rospy.Service('get_demonstration', GetDemonstration, self.servicecb_get_demonstration)
        # self.service = rospy.Service('fetch_demo_count', DemoCount, self.servicecb_demo_count)

    # def servicecb_demo_count(self, msg):
    #     dir = self.dir_template.format(msg.name)
    #     if not os.path.exists(dir):
    #         return DemoCountResponse(count=0)
        
    #     count = len([name for name in os.listdir(dir) if os.path.isfile(os.path.join(dir, name))])
    #     return DemoCountResponse(count=count)


    # def format_filename(self, filename):
    #     # Split text (demo base name) and number (demo id) from  the original message
    #     temp = re.compile("([a-zA-Z]+)([0-9]+)")
    #     name = temp.match(filename).groups()

    #     # Check if the directory exists
    #     dir = self.dir_template.format(name[0])
    #     if not os.path.exists(dir):
    #         os.makedirs(dir)

    #     return self.filename_template.format(name[0],name[1])


    def subcb_save_trajectory(self, demo_msg : DemonstrationMsg):
        """
        Saves a new demonstration and its serialized trajectory to the database using SQLAlchemy.

        Args:
            demo_msg (DemonstrationMsg): The ROS demonstration message.
        """
        try:
            # Resolve robot ID from robot name
            robot = self.session.query(RobotDB).filter_by(name=demo_msg.robot_name).first()
            if not robot:
                rospy.logerr(f"Robot with name '{demo_msg.robot_name}' does not exist.")
                return

            # Create a new demonstration entry
            demo = DemonstrationDB(
                name=demo_msg.name,
                robot_id=robot.id,
                meta_data={"description": demo_msg.description} 
            )
            self.session.add(demo)
            self.session.commit()

            # Serialize the demonstration message and save the trajectory
            pickled_demo = pickle.dumps(demo_msg)
            trajectory = TrajectoryDB(
                demo_id=demo.id,
                type=demo_msg.trajectory_type,
                trajectory_data=pickled_demo
            )
            self.session.add(trajectory)
            self.session.commit()

            rospy.loginfo("Demonstration and trajectory saved successfully.")

        except Exception as e:
            self.session.rollback()
            rospy.logerr(f"Failed to save demonstration: {e}")

        finally:
            self.session.close()

    def servicecb_get_demonstration(self, req : GetDemonstrationRequest):
        try:
            # Retrieve the demonstration by name and robot name
            demo = (
                self.session.query(DemonstrationDB)
                .join(RobotDB, DemonstrationDB.robot_id == RobotDB.id)
                .filter(DemonstrationDB.name == req.name, RobotDB.name == req.robot_name)
                .first()
            )
            
            if not demo:
                rospy.logwarn(f"No demonstration found with name: {req.name} for robot: {req.robot_name}")
                return GetDemonstrationResponse(success=False, message=f"No demonstration found with name: {req.name} for robot: {req.robot_name}")
            
            # Retrieve the associated trajectory of the requested type
            trajectory = (
                self.session.query(TrajectoryDB)
                .filter_by(demo_id=demo.id, type=req.trajectory_type)
                .first()
            )
            
            if not trajectory:
                rospy.logwarn(f"No trajectory of type {req.trajectory_type} found for demonstration: {req.name} and robot: {req.robot_name}")
                return GetDemonstrationResponse(success=False, message=f"No trajectory of type {req.trajectory_type} found for demonstration: {req.name} and robot: {req.robot_name}")
                       
            # Deserialize the trajectory data
            demo_msg = pickle.loads(trajectory.trajectory_data)
            
            # Construct the response
            response = GetDemonstrationResponse(
                success=True,
                message="Demonstration retrieved successfully",
                Demonstration=demo_msg
            )
            
            return response
        
        except Exception as e:
            rospy.logerr(f"Failed to retrieve demonstration: {e}")
            return GetDemonstrationResponse(success=False, message=f"Error retrieving demonstration: {str(e)}")
        
        finally:
            self.session.close()

