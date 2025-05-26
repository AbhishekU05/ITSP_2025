import pybullet as p
import pybullet_data
import numpy as np
import time
import random

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane = p.loadURDF("plane.urdf")

# Parameters
num_bots = 10
bots = []
radius = 0.05
height = 0.1
start_area = 2.0
region_radius = 0.5

# Create 10 bots with identical color
for _ in range(num_bots):
    start_x = random.uniform(-start_area, start_area)
    start_y = random.uniform(-start_area, start_area)
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[0.3, 0.7, 1, 1])
    bot_id = p.createMultiBody(baseMass=1,
                               baseCollisionShapeIndex=col,
                               baseVisualShapeIndex=vis,
                               basePosition=[start_x, start_y, height / 2])
    bots.append(bot_id)

target_position = np.array([0, 0])

# ---- Mouse Interaction ----

def get_mouse_click_position():
    keys = p.getKeyboardEvents()
    if ord('g') in keys and keys[ord('g')] & p.KEY_WAS_TRIGGERED:
        cam = p.getDebugVisualizerCamera()
        width, height, view_matrix, proj_matrix = cam[0], cam[1], cam[2], cam[3]

        mouse_events = p.getMouseEvents()
        for event in mouse_events:
            if event[0] == p.MOUSE_BUTTON_LEFT and event[3] & p.KEY_WAS_TRIGGERED:
                mouse_x = event[1]
                mouse_y = event[2]
                ray_start, ray_end = p.computeViewRay(mouse_x, mouse_y)
                hit = p.rayTest(ray_start, ray_end)[0]
                if hit[0] != -1:
                    return np.array(hit[3][:2])
    return None

# ---- Helper ----

def get_bot_positions():
    return np.array([p.getBasePositionAndOrientation(bot)[0][:2] for bot in bots])

def compute_priority_order(positions, target):
    dists = [np.linalg.norm(pos - target) for pos in positions]
    return np.argsort(dists)  # bots closer to target have higher priority

def move_bots(positions, target, priorities):
    new_vels = []
    for i, pos in enumerate(positions):
        if np.linalg.norm(pos - target) < region_radius:
            vel = np.array([0, 0])
        else:
            direction = (target - pos)
            norm = np.linalg.norm(direction)
            direction = direction / norm if norm > 0 else np.zeros_like(direction)
            vel = direction

            # Repulsion from higher-priority bots
            for j in priorities[:np.where(priorities == i)[0][0]]:
                other_pos = positions[j]
                dist = np.linalg.norm(pos - other_pos)
                if dist < 0.3 and dist > 1e-2:
                    repulsion = (pos - other_pos) / dist * 0.5
                    vel += repulsion

        vel = np.clip(vel, -1, 1)
        new_vels.append(vel)

    for bot, vel in zip(bots, new_vels):
        current_z = p.getBasePositionAndOrientation(bot)[0][2]
        p.resetBaseVelocity(bot, linearVelocity=[vel[0], vel[1], 0])

# ---- Main Loop ----

while True:
    click = get_mouse_click_position()
    if click is not None:
        target_position = click
        print("New target:", target_position)

    bot_positions = get_bot_positions()
    priority_order = compute_priority_order(bot_positions, target_position)
    move_bots(bot_positions, target_position, priority_order)

    p.stepSimulation()
    time.sleep(1.0 / 240)

