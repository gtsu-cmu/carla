import argparse
import carla
import ipfshttpclient
import subprocess
import paramiko
import re
import queue
import numpy as np
import random
import time

client_IP = 'localhost'
server_IP = ""
server_username = "riope"
pkey_path = "C:\\Users\\akars\\.ssh\\id_rsa"

class sim:
	def __init__(self, args) -> None:
		self.client = carla.Client(client_IP, args.port)
		self.client.set_timeout(10.0)

		if args.town == '':
			self.world = self.client.get_world()
		else:
			self.world = self.client.load_world('Town'+args.town)
		self.bp_lib = self.world.get_blueprint_library()
		self.spawn_pts = self.world.get_map().get_spawn_points()
		self.actorList = list()

		settings = self.world.get_settings()
		settings.fixed_delta_seconds = args.time_step
		self.world.apply_settings(settings)

	def setup_env(self, args):
		vehicle_bp = self.bp_lib.find('vehicle.' + args.vehicle)
		vehicle = self.world.try_spawn_actor(vehicle_bp, random.choice(self.spawn_pts))
		self.actorList.append(vehicle)

		for i in range(args.npcs):
			vehicle_bp = random.choice(self.bp_lib.filter('vehicle'))
			npc = self.world.try_spawn_actor(vehicle_bp, random.choice(self.spawn_pts))
			if npc is not None:
				self.actorList.append(npc)

		for v in self.world.get_actors().filter('*vehicle*'):
			v.set_autopilot(True)

		return vehicle

	def setup_sensors(self, args, vehicle):
		cam_bp = self.bp_lib.find('sensor.camera.rgb')
		cam_init = carla.Transform(carla.Location(x=0.4, z=1.6))

		# cam_bp.set_attribute('image_size_x', str(80))
		# cam_bp.set_attribute('image_size_y', str(80))
		# cam_bp.set_attribute('sensor_tick', str(args.time_step))

		cam = self.world.spawn_actor(cam_bp, cam_init, attach_to=vehicle)
		self.actorList.append(cam)
		return cam
	
	def destroy(self):
		self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actorList])

class ShellHandler:
    def __init__(self, host, user, path):
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(host, username=user, key_filename=path, port=22)

        channel = self.ssh.invoke_shell()
        self.stdin = channel.makefile('wb')
        self.stdout = channel.makefile('r')

    def __del__(self):
        self.ssh.close()

    def execute(self, cmd):
        cmd = cmd.strip('\n')
        self.stdin.write(cmd + '\n')
        finish = 'end of stdOUT buffer. finished with exit status'
        echo_cmd = 'echo {} $?'.format(finish)
        self.stdin.write(echo_cmd + '\n')
        shin = self.stdin
        self.stdin.flush()

        shout = []
        sherr = []
        exit_status = 0
        for line in self.stdout:
            if str(line).startswith(cmd) or str(line).startswith(echo_cmd):
                # up for now filled with shell junk from stdin
                shout = []
            elif str(line).startswith(finish):
                # our finish command ends with the exit status
                exit_status = int(str(line).rsplit(maxsplit=1)[1])
                if exit_status:
                    sherr = shout
                    shout = []
                break
            else:
                # get rid of 'coloring and formatting' special characters
                shout.append(re.compile(r'(\x9B|\x1B\[)[0-?]*[ -/]*[@-~]').sub('', line).
                             replace('\b', '').replace('\r', '').replace('\n',''))

        # first and last lines of shout/sherr contain a prompt
        if shout and echo_cmd in shout[-1]:
            shout.pop()
        if shout and cmd in shout[0]:
            shout.pop(0)
        if sherr and echo_cmd in sherr[-1]:
            sherr.pop()
        if sherr and cmd in sherr[0]:
            sherr.pop(0)

        return shin, shout, sherr

    def exec_cmd(self, cmd):
        try:
            _, std_out, _ = self.execute(cmd)
            for output in std_out:
                print(output)
			
        except Exception as error_message:
            print("Couldn't run command")
            print(error_message)

def parse_args():
	parser = argparse.ArgumentParser(description='Describe the simulation environment')

	call_args = parser.add_argument_group("Execution options")
	call_args.add_argument('-r', '--run', action='store_true', help='Runs the CARLA simulation server')
	call_args.add_argument('-s', '--simulate', action='store_true', help='Sets the simulation environment')
	call_args.add_argument('--invoke', action='store_true', help='Specifies function to invoke')
	call_args.add_argument('--query', choices=["ReadFrameData", "ReadVehicleFrames"], help='Specifies function to invoke')

	sim_args = parser.add_argument_group("Simulation parameters")
	sim_args.add_argument('--town', action='store', type=str, default='', help='Town to be stimulated')
	sim_args.add_argument('--vehicle', action='store', type=str, default='lincoln.mkz_2020', help='Name of the vehicle to be monitored')
	sim_args.add_argument('--vid', action='store', type=int, default='1', help='ID of the monitored vehicle')
	sim_args.add_argument('--frames', action='store', type=int, default='5', help='Number of frames to be monitored')
	sim_args.add_argument('--timestamp', action='store', type=float, default='11.1', help='Time when data is to be retrieved')
	sim_args.add_argument('-n', '--npcs', action='store', type=int, default='30', help='Number of vehicles in the environment')
	sim_args.add_argument('-t', '--time_step', action='store', type=float, default='0.05', help='Time step for simulation')
	sim_args.add_argument('-p', '--port', action='store', type=int, default='2000', help='Port to connect client')

	return parser.parse_args()

def exec_cmd(command):
	proc = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	out, err = proc.communicate()
	return out.decode()

def run_sim(args):
	arg_list = list()
	arg_list.append("-carla-port=" + str(args.port))
	proc = subprocess.Popen(["C:\\Users\\akars\\CARLA\\CarlaUE4.exe"] + arg_list)
	time.sleep(20)
	return proc

def camera_callback(image, data_list, frames):
	if frames[0] <= 0:
		return
	
	output = {'frame': 0, 'timestamp': 0.0, 'image': np.zeros((image.height, image.width, 4))}
	output['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
	output['frame'] = image.frame
	output['timestamp'] = image.timestamp

	data_list.put(output, block=True)
	frames[0] = frames[0] -1

def main():
	args = parse_args()

	if args.invoke and not args.simulate:
		raise Exception("Can't push data without simulating")

	if args.run:
		proc = run_sim(args)

	if args.simulate:
		env = sim(args)
		vehicle = env.setup_env(args)
		cam = env.setup_sensors(args, vehicle)

		if args.invoke:
			frames = [args.frames]
			data_list = queue.Queue()
			cam.listen(lambda image: camera_callback(image, data_list, frames))
			# time.sleep(5)
			# cam.listen(lambda image: image.save_to_disk('out/%06d.png' % image.frame))

			client = ipfshttpclient.connect('/ip4/127.0.0.1/tcp/5001/http')
			server_IP = exec_cmd("wsl hostname -I").strip()
			obj = ShellHandler(server_IP, server_username, pkey_path)

			i = 1
			while not data_list.empty():
				cam_data = data_list.get(block=True)

				# Call invoke PushData
				try:
					lst = cam_data['image'].tolist()
					cid = client.add_json(lst)
					print(cid)
				except Exception as error_message:
					print("Couldn't run command")
				
				str_list = ["node", "invoke.js", "PushData", str(args.vid), str(cam_data['timestamp']),\
							 str(cam_data['frame']), cid]
				cmd = " ".join(str_list)

				obj.exec_cmd(cmd)
				print("Uploaded frame " + str(i))
				print()
				i = i + 1
			
			cam.stop()

		env.destroy()
		if args.run:
			proc.kill()
	
	if args.query != None:
		server_IP = exec_cmd("wsl hostname -I").strip()
		obj = ShellHandler(server_IP, server_username, pkey_path)

		if args.query == "ReadFrameData":
			str_list = ["node", "invoke.js", "ReadFrameData", str(args.vid), str(args.timestamp)]
			cmd = (" ").join(str_list)
			obj.exec_cmd(cmd)

		else:
			str_list = ["node", "invoke.js", "ReadVehicleFrames", str(args.vid)]
			cmd = (" ").join(str_list)
			obj.exec_cmd(cmd)

if __name__=="__main__":
    main()