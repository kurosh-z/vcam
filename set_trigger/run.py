from __future__ import print_function
import subprocess

def execute(cmd):
    popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
    for stdout_line in iter(popen.stdout.readline, ""):
        yield stdout_line 
    popen.stdout.close()
    return_code = popen.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, cmd)
   

if __name__ =="__main__":
 
    # for path in execute(["roslaunch", "/home/kurosh/catkin-ws/src/vcam/launch/vcam_nodelet_launch.launch"]):
    for path in execute(["sudo", "/home/kurosh/catkin-ws/src/vcam/set_trigger/set_trigger"]):
        print(path, end="")
    


