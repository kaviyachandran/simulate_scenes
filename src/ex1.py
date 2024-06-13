# import os
#
# import mujoco as mj
# import mediapy as media
#
# initial_file_path = "world_two_cups.xml"
# # Create MuJoCo context
# path_to_xml = os.path.abspath(initial_file_path)
# print(path_to_xml)
#
# # Make model and data
# model = mj.MjModel.from_xml_path(path_to_xml)
# data = mj.MjData(model)
#
# # Make renderer, render and show the pixels
# renderer = mj.Renderer(model)
# media.show_image(renderer.render())


import mujoco as mj
import mujoco_viewer

model = mj.MjModel.from_xml_path('../model/world_two_cups.xml')
data = mj.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)


def controller(model, data):
	dest_name = "dest_cup_joint"
	jnt_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, dest_name)
	dof_adr = model.jnt_dofadr[jnt_id]
	qpos_adr = model.jnt_qposadr[jnt_id]
	dt = model.opt.timestep
	print(data.qpos[qpos_adr + 2])
	time_for_particles_to_settle = 2.53
	# if data.time() > 2.53:
	data.qpos[qpos_adr + 2] = data.qpos[qpos_adr + 2] + 0.1 * dt


mj.set_mjcb_control(controller)

# # simulate and render
for _ in range(10000):
	if viewer.is_alive:
		mj.mj_step(model, data)
		viewer.render()
	else:
		break

# close
viewer.close()
