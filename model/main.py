# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import os
import mujoco as mj
import numpy as np
import glfw

# Initialize GLFW
if not glfw.init():
    raise Exception("GLFW cannot be initialized!")

# Create a GLFW window
window = glfw.create_window(640, 480, "MuJoCo Simulation", None, None)
if not window:
    glfw.terminate()
    raise Exception("GLFW window cannot be created!")

# Set the current context to the window
glfw.make_context_current(window)
initial_file_path = "world_two_cups.xml"

# Create MuJoCo context
path_to_xml = os.path.abspath(initial_file_path)
print(path_to_xml)
model = mj.MjModel.from_xml_path(path_to_xml)
data = mj.MjData(model)


gl_context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# Initialize MuJoCo scene
scene = mj.MjvScene(model, maxgeom=10000)
cam = mj.MjvCamera()

# Set up camera parameters
cam.azimuth = 60  # horizontal angle measured from a baseline
cam.elevation = -20  # angle measured from the x-y plane
cam.distance = 2.0
cam.lookat[0] = 2
cam.lookat[1] = 0
cam.lookat[2] = 0.3

duration = 3  # seconds
framerate = 60  # frames per second
framecount = 0
sim_timestep = 1 / 250

jnt_name = "dest_joint"


def controller(model, data):
    jnt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jnt_name)
    addr = model.jnt_qposadr[jnt_id]
    if data.time > 0.7:
        data.qpos[addr+2] = 0.8


mj.set_mjcb_control(controller)

# simstart = data.time
# Simulation loop
while data.time < duration:
    # Step the simulation
    mj.mj_step(model, data)
    # print("date time: ", data.time, framerate * data.time)
    # Render the scene  and update the window
    if framecount < framerate * data.time:
        framecount += 1
        # print("framecount: ", framecount)
        # Update the scene
        mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)

        # Render the scene in the window
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        mj.mjr_render(viewport, scene, gl_context)

        # Swap buffers and poll events
        glfw.swap_buffers(window)
        glfw.poll_events()

# Clean up
glfw.terminate()

# Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     simulate("")
