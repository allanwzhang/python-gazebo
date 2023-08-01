# Ardupilot-Python-Gazebo Framework Instructions
## Install Ardupilot
Follow the installArdupilot.md file in this repository for your Linux version: [18.04](https://github.com/allanwzhang/python-gazebo/blob/main/installArdupilot18.md), [20.04](https://github.com/allanwzhang/python-gazebo/blob/main/installArdupilot20.md)

## Install Python-Gazebo
```bash
git clone https://github.com/allanwzhang/python-gazebo.git
cd python-gazebo
git checkout gym-env
```
Follow the [installPythonGazebo.md](https://github.com/allanwzhang/python-gazebo/blob/main/installPythonGazebo.md) file to install all required parts (also in the gym-env branch)

## Commands to run code
### Start Ardupilot
From the ardupilot directory
```bash
cd ArduCopter
sim_vehicle.py -v ArduCopter -f octa --model JSON --console
# may need run conda deactivate first
```
Note: Make sure X forwarding application is on (if running on windows os)
### Start Gazebo
From python-gazebo directory
```bash
gazebo worlds/test.world --verbose -u
```
### Start Python
From python-gazebo directory, open up env/APDemo.ipynb file and run blocks to start the connection.
### Control Parameters
After all three parts are running, load Octocopter parameters from the ArduPilot console. There should be an parameters button where you can then choose to load a file. Choose simOcto.parm in the python-gazebo/env directory. You will only need to do this once (parameters should save automatically).
### Starting the drone
In the console where Ardupilot is running, enter these commands after everything loads and connects. Look for these two commands to appear in the ArduPilot console: IMU0 is using GPS, IMU1 is using GPS, then run these commands:
```bash
mode guided
arm throttle
takeoff 10
```
The drone should then takeoff in the Gazebo window
### Using a ground control system
To run more advanced flights, you can download a Ground Control system.  
IQ_GNC Tutorial: [QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_qgc.md)  
Instead of running commands from terminal, you can set flights from the ground control system instead  
Note: This feature is not tested yet
