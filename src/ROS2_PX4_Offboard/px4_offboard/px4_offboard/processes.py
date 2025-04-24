#!/usr/bin/env python3
import subprocess
import time


def main(args=None):
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                    "MicroXRCEAgent udp4 -p 8888" + "; exec bash"]) # start the agent
    
    time.sleep(1)

    cd_command = "cd ~/PX4-Autopilot-ColAvoid && " # path to PX4 repository
    # pose = "-30,40,0,0,0,0"
    pose = "0,0,0,0,0,0"
    simulation_command = f"PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=default PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE={pose} ./build/px4_sitl_default/bin/px4"
                         # port                   world                 model

    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                    cd_command + simulation_command + "; exec bash"]) # spawn the UAV

if __name__ == '__main__':
    main()