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

llm = ChatOpenAI(model="gpt-4o", temperature=1)

#%%
from pydantic import BaseModel, Field
from langchain_core.messages import HumanMessage, SystemMessage

class DecomposeQuery(BaseModel):
    subtasks: list[str] = Field(None, 
                                description="Decompose a given robotic task into the minimal number of essential robot arm movements to facilitate user demonstration. - Each subtask should represent a single, necessary motion.- Ensure efficiency by minimizing redundancy and excluding non-essential actions.- If two subsequent subtasks can be effectively demonstrated together, combine them.- Ensure the list allows for task demonstration one by one.- one demonstration can include lifting and moving at once"
                                )

structured_llm = llm.with_structured_output(DecomposeQuery)




output = structured_llm.invoke(
            [
                # SystemMessage(
                #     content="Decompose the given task into the minimal number of continuous robot arm movements. Each subtask should involve a single uninterrupted motion. Only include essential movements. Do NOT include intermediate steps. DO NOT include stops to clear from ostacles or any steps regarding safety"
                # ),
                HumanMessage(content="here is the description of my task. I have a screw, and o-ring, and an auxilliary cone. first I need to pick a screw and mount on a fixture. then I need to pick up the auxilliary cone and mount it on top of the screw. then, I need to pick an o-ring and slide it down the auxilliary cone to mount it on the screw. finally, I need to remove the cone and put it back, and then remove the mounted screw as well.")
            ]
        )

#%%
output.subtasks