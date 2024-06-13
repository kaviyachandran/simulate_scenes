import os

import mujoco as mj
import mediapy as media


initial_file_path = "../model/world_two_cups.xml"
# Create MuJoCo context
path_to_xml = os.path.abspath(initial_file_path)
print(path_to_xml)

# Make model and data
model = mj.MjModel.from_xml_path(path_to_xml)
data = mj.MjData(model)
# Make renderer, render and show the pixels
renderer = mj.Renderer(model)

duration = 3.8  # (seconds)
framerate = 60  # (Hz)

# Simulate and display video.
frames = []
mj.mj_resetData(model, data)  # Reset state and time.
while data.time < duration:
  mj.mj_step(model, data)
  if len(frames) < data.time * framerate:
    renderer.update_scene(data)
    pixels = renderer.render()
    frames.append(pixels)

media.show_video(frames, fps=framerate)
renderer.close()

