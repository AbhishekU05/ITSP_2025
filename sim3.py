import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize simulation with GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Camera view (top-down)
p.resetDebugVisualizerCamera(cameraDistance=3,
                             cameraYaw=0,
                             cameraPitch=-89.999,
                             cameraTargetPosition=[0, 0, 0])

# Parameters
num_bots = 3
bot_radius = 0.1
bot_height = 0.2
min_dist = 0.2  # Minimum separation distance (20 cm)
speed = 0.02
start_positions = [(-1, -1), (0, -1), (1, -1)]
target_positions = [(1, 1), (0, 1), (-1, 1)]
bot_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
bot_ids = []

# Create bots with different colors
for i, pos in enumerate(start_positions):
    colShape = p.createCollisionShape(p.GEOM_CYLINDER, radius=bot_radius, height=bot_height)
    visShape = p.createVisualShape(p.GEOM_CYLINDER, radius=bot_radius, length=bot_height,
                                   rgbaColor=bot_colors[i])
    bot_id = p.createMultiBody(baseMass=1,
                               baseCollisionShapeIndex=colShape,
                               baseVisualShapeIndex=visShape,
                               basePosition=[pos[0], pos[1], bot_height/2])
    bot_ids.append(bot_id)

# Get bot positions (simulate camera tracking)
def get_bot_positions(bot_ids):
    return np.array([p.getBasePositionAndOrientation(bid)[0][:2] for bid in bot_ids])

def move_bots(bot_ids, targets, speed, min_dist):
    positions = get_bot_positions(bot_ids)
    for i, bot_id in enumerate(bot_ids):
        current = positions[i]
        target = np.array(targets[i])
        direction = target - current
        dist_to_target = np.linalg.norm(direction)
        if dist_to_target < 0.01:
            continue
        direction = direction / dist_to_target

        repulsion = np.zeros(2)
        too_close = False
        for j, other_pos in enumerate(positions):
            if i != j:
                diff = current - other_pos
                dist = np.linalg.norm(diff)
                if dist < min_dist and dist > 0:
                    too_close = True
                    repulsion += (diff / dist) * (1 - dist / min_dist)

        # If too close, prioritize repulsion and reduce forward speed
        if too_close:
            move_vec = repulsion
            # Normalize repulsion and scale speed down (e.g., 50%)
            move_vec = move_vec / np.linalg.norm(move_vec) * (speed * 0.5)
        else:
            # Normal move + slight repulsion to keep distance
            move_vec = direction + repulsion * 0.5
            move_vec = move_vec / np.linalg.norm(move_vec) * speed

        new_pos = current + move_vec
        p.resetBasePositionAndOrientation(bot_id, [new_pos[0], new_pos[1], bot_height / 2], [0, 0, 0, 1])



# Simulation loop
for _ in range(1000):
    move_bots(bot_ids, target_positions, speed, min_dist)
    p.stepSimulation()
    time.sleep(1. / 240.)

# Get final positions
final_positions = get_bot_positions(bot_ids)
p.disconnect()
final_positions

