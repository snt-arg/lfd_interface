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
from langgraph.graph import StateGraph
from sentence_transformers import SentenceTransformer, util

# Load a model for semantic similarity
model = SentenceTransformer("all-mpnet-base-v2")

#%%
# Function to find the best matching subtask using semantic similarity
def find_best_match(decomposed_task: str, known_subtasks: dict[str, str]) -> tuple[str, float]:
    known_descriptions = list(known_subtasks.values())
    embeddings1 = model.encode(decomposed_task, convert_to_tensor=True)
    embeddings2 = model.encode(known_descriptions, convert_to_tensor=True)
    similarities = util.pytorch_cos_sim(embeddings1, embeddings2).squeeze()
    best_idx = similarities.argmax().item()
    return list(known_subtasks.keys())[best_idx], similarities[best_idx].item()
#%%
decomposed_subtasks = ['Move to screw location.',
 'Grasp the screw.',
 'Lift the screw.',
 'Move to the fixture location with the screw.',
 'Align and mount the screw onto the fixture.',
 'Release the screw from the gripper.',
 'Move to auxiliary cone location.',
 'Grasp the auxiliary cone.',
 'Lift the auxiliary cone.',
 'Move to the screw mounted on the fixture.',
 'Align the auxiliary cone above the screw.',
 'Lower the auxiliary cone onto the screw.',
 'Release the cone from the gripper.',
 'Move to O-ring location.',
 'Grasp the O-ring.',
 'Lift the O-ring.',
 'Move to the mounted auxiliary cone.',
 'Align the O-ring above the cone.',
 'Lower and slide the O-ring down the auxiliary cone to mount it on the screw.',
 'Release the O-ring from the gripper.',
 'Grasp the auxiliary cone.',
 'Lift the auxiliary cone off the screw.',
 'Move back to the auxiliary cone storage location.',
 'Release and position the auxiliary cone.',
 'Move to the fixture-mounted screw.',
 'Grasp the mounted screw.',
 'Lift the screw off the fixture.',
 'Move to the original screw location.',
 'Release and position the screw.',
 'Return to home/start position.']
known_subtasks = {"reach_screw": "move from home location to reach and pick up the screw",
                  "move_screw_fixture": "move the screw to the fixture and mount the screw",
                  "move_fixture_cone": "move from the fixture to the cone station pick up the auxilliary cone",
                  "lift_cone_station": "lift the auxilliary cone from the cone station",
                  "move_cone_fixture": "move the cone from the cone station to the fixture",
                  "mount_cone_fixture": "mount the cone on top of the screw",
                  "move_fixture_ring": "move from the fixture to the o-ring location and pick it up",
                  "move_ring_fixture": "move the ring to the fixture location on top of the auxilliary cone",
                  "slide_ring_screw": "slide the o-ring on the auxilliary cone to mount it on the screw",
                  "lift_cone_fixture": "lift the auxilliary cone from the fixture",
                  "move_cone_station": "move the auxilliary cone to the cone station",
                  "move_station_fixture": "move from the cone station to fixture to pick up the screw",
                  "move_screw_final": "move the screw to the final station"}
for subtask in decomposed_subtasks:
    best_match, confidence = find_best_match(subtask, known_subtasks)
    print (best_match, confidence)