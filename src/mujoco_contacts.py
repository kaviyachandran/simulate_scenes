import os
import mujoco
import numpy as np
import cv2
from dm_control import mujoco as m

n_frames = 10000
frame_rate = 10
height = 480
width = 640

def create_videos(images):
    for image in images:
        cv2.imshow('Frame', image)
        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

PATH_TO_HUMANOID_XML = os.path.expanduser('~/software/mujoco_menagerie/pr2/pr2.xml')

# Load the model and make a simulator
model = mujoco.MjModel.from_xml_path(PATH_TO_HUMANOID_XML)
data = mujoco.MjData(model)

# mujoco.mj_step(model, data, nstep=50)


physics = m.Physics.from_xml_path(PATH_TO_HUMANOID_XML)

frames = np.zeros((n_frames // frame_rate, height, width, 3), dtype=np.uint8)
for i in range(n_frames):
    physics.step()
    if i % frame_rate == 0 and i > 0:
        frames[i // frame_rate] = physics.render(height, width)
create_videos(frames)

# # Simulate 1000 steps so humanoid has fallen on the ground
for _ in range(10):
    mujoco.mj_step(model,data)

print('number of contacts',data.ncon)
for i in range(data.ncon):
    # Note that the contact array has more than `ncon` entries,
    # so be careful to only read the valid entries.
    contact = data.contact[i]
    print('contact', i)
    print('dist', contact.dist)
    print('point ', contact.pos)
    print('geom1', contact.geom1, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, contact.geom1))
    print('geom2', contact.geom2, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, contact.geom2))
    # # There's more stuff in the data structure
    # # See the mujoco documentation for more info!
    geom2_body = model.geom_bodyid[data.contact[i].geom2]
    print(' Contact force on geom2 body', data.cfrc_ext[geom2_body])
    print('norm', np.sqrt(np.sum(np.square(data.cfrc_ext[geom2_body]))))
    # Use internal functions to read out mj_contactForce
    c_array = np.zeros(6, dtype=np.float64)
    # print('c_array', c_array)
    mujoco.mj_contactForce(model, data, i, c_array)
    print('c_array', c_array)

print('done')
