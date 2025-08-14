#!/usr/bin/env python3
import subprocess
import time


PX4_PATH = "~/PX4-Autopilot-ColAvoid"
WORLD_NAME = "building"


def main(args=None):
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                    "MicroXRCEAgent udp4 -p 8888" + "; exec bash"]) # start the agent
    
    time.sleep(1)

    cd_command = f"cd {PX4_PATH} && " # path to PX4 repository
    pose = "-30,40,0,0,0,0"
    simulation_command = f"PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD={WORLD_NAME} PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE={pose} ./build/px4_sitl_default/bin/px4"
                         # port                   world                     model                       pose

    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                    cd_command + simulation_command + "; exec bash"]) # spawn the UAV

if __name__ == '__main__':
    main()