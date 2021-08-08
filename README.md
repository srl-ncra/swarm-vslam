# Swarm-vSLAM: A Swarm Multi-mapping Visual SLAM
## Project Flow Diagram
![Flowchart](https://github.com/srl-ncra/swarm-vslam/blob/main/Flowchart.jpg?raw=true)
## 1. Client Setup (over SIMBot)
We did the client setup on a NanoPi NEO Plus 2 which was operating Ubuntu Core 20.04 LTS. You can set up a NanoPi by following this tutorial: [NanoPi NEO Plus2](https://wiki.friendlyarm.com/wiki/index.php/NanoPi_NEO_Plus2). 
After Ubuntu installation, assign an IP to the NanoPi and access it using ssh, then follow these commands:

1. Make sure that you are in root directory.

2. Run: `mkdir opencv_com`
3. Run: `cd opencv_com`

4. Run: `sudo nano requirements.txt`
	Copy the text from requirements.txt to this file and save.

5. Run: `pip install -r requirements.txt` (All libraries will be automatically installed)

6. Run: `sudo nano obs_pi.py`
	Copy content of obs_pi.py to this file.
	Save and close. (C+S then C+X)

### If you face following OpenCV error  
	
Error: ImportError: libGL.so.1: cannot open shared object file: No such file or directory

1. Run: `sudo apt-get update && apt-get install -y python3-opencv`

### Running Obstacle Detection 
This code will start moving the SIMBot while avoiding obstacles and publishing camera data on a ROS topic which the server will subscribe to
1. Make sure you are in opencv_com directory.
2. Run: `python3 obs_pi.py`

  
## 2. Server Setup (GUI for visualization) 
To setup the server, we need to install [CCM-SLAM](https://github.com/VIS4ROB-lab/ccm_slam) on the PC
