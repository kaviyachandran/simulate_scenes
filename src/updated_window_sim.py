# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import os
import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from helper_util import lerp, distance

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


def mouse_button(window, button, act, mods):
    # update button state
    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx / height,
                      dy / height, scene, cam)


def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


initial_file_path = "world_two_cups.xml"

# Create MuJoCo context
path_to_xml = os.path.abspath(initial_file_path)
print(path_to_xml)
model = mj.MjModel.from_xml_path(path_to_xml)
data = mj.MjData(model)
cam = mj.MjvCamera()  # Abstract camera
opt = mj.MjvOption()

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
# Initialize MuJoCo scene
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)
# Set up camera parameters
cam.azimuth = 0  # horizontal angle measured from a baseline
cam.elevation = -5  # angle measured from the x-y plane
cam.distance = 5.0
cam.lookat[0] = 2
cam.lookat[1] = 0
cam.lookat[2] = 0.3
simend = 20
initial_time_for_particles_to_settle = 0.7


def controller(model, data):
    dest_name = "dest_cup_joint"
    jnt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, dest_name)
    qpos_adr = model.jnt_qposadr[jnt_id]
    # print(data.qpos[qpos_adr+2])
    # data.qvel[dof_adr+2] = 0.1
    # print(data.time)
    # dest_pos = data.qpos[qpos_adr:qpos_adr + 2]
    #
    # start = [src_pose[0], src_pose[1] - src_dim[1], src_pose[2] + src_dim[2] / 2]
    # dist = distance(start, dest_pos)
    # t_req = dist / 0.1
    # total_timesteps = t_req / model.opt.timestep
    # num = total_timesteps
    # data.ctrl[:] =
    if data.time > initial_time_for_particles_to_settle:
        #     positions = lerp(dest_pos, start, num)
        #     print(len(positions), positions[0])
        #     for i in range(int(total_timesteps)):
        #         data.qpos[qpos_adr] = positions[i][0]
        #         data.qpos[qpos_adr+1] = positions[i][1]
        #         data.qpos[qpos_adr+2] = positions[i][2]
        #         num = num - 1
        #     if num == 0:
        data.qpos[qpos_adr + 2] = 0.8


mj.set_mjcb_control(controller)
framerate = 30
framecount = 0

src_jnt_name = "src_cup_joint"
src_dim = (0.2, 0.2, 0.12)
src_jnt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, src_jnt_name)
src_jnt_qpos_adr = model.jnt_qposadr[src_jnt_id]
src_pose = data.qpos[src_jnt_qpos_adr:src_jnt_qpos_adr + 7]

# model.opt.timestep = 0.008
### generalized pose

# Simulation loop
while not glfw.window_should_close(window):
    simstart = data.time
    while data.time - simstart < (1.0 / 500.0):
        # print("current_position: ", data.qpos[jnt_id*7+3], data.qpos[jnt_id*7+4], data.qpos[jnt_id*7+5], data.qpos[jnt_id*7+6])
        # data.qpos[jnt_id*7+3] = 0.7071
        # data.qpos[jnt_id*7+4] = 0.0
        # data.qpos[jnt_id*7+5] = 0
        # data.qpos[jnt_id*7+6] = 0.7071
        # temp = data.xpos[body_id*3]
        # data.qpos[body_id*7+1] += 0.01
        # Step the simulation
        mj.mj_step(model, data)
        mj.mjv_updateScene(model, data, opt, None, cam,
                           mj.mjtCatBit.mjCAT_ALL.value, scene)

    if data.time >= simend:
        break

    if data.time * framerate > framecount:
        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(
            window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        # Update scene and render

        mj.mjr_render(viewport, scene, context)

        # swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(window)

        # process pending GUI events, call GLFW callbacks
        glfw.poll_events()
        framecount += 1

# Clean up
glfw.terminate()

# Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     simulate("")
