#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  8 17:06:37 2025

@author: abrk
"""

import os
# Configure OpenAI API Key
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

from langchain_openai import ChatOpenAI

llm = ChatOpenAI(model="gpt-4o")

#%%
# Function to validate the demonstration name
def validate_demo_name(name):
    if " " in name or any(char.isdigit() for char in name):
        return False
    return True


# Function to construct the roslaunch command
def construct_roslaunch_command(robot_name, demo_name, description):
    return f"roslaunch lfd_interface lfd_recorder.launch robot_name:={robot_name} name:={demo_name} description:=\"{description}\""

# Function to execute the command
def execute_command(command):
    print(f"Executing command: {command}")
    os.system(command)

#%%
from pydantic import BaseModel, Field

class DemoRecordQuery(BaseModel):
    robot_name: str = Field(None, description="The name of the robot, possible options are [fr3,yumi_l,yumi_r]")
    description: str = Field(
        None, description="a short and concise description about the task"
    )
    demo_name: str = Field(
        None, description="the demonstration name. if the demonstratio name is not given explicitly, Generate a concise, unique name for a robot demonstration based on the description. Use only lowercase letters and underscores, no spaces or numbers."
    )

structured_llm = llm.with_structured_output(DemoRecordQuery)

output = structured_llm.invoke("Record a demonstration of picking up a coffee cup with the FR3 robot")

#%%
construct_roslaunch_command(output.robot_name, output.demo_name, output.description)
