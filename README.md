# Instructions to setup Ardupilot-Python-Gazebo Framework
## Install Ardupilot
IQ_GNC Tutorial: [Ubuntu 18.04](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md)  
                 [Ubuntu 20.04](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)  
Note: Checkout latest stable ArduCopter version (I used Copter 4.2.3)  

## Install Python-Gazebo
```bash
git clone https://github.com/allanwzhang/python-gazebo.git
git checkout gym-env
```
Follow instructions in gym-env readme to install all required parts

## Commands to run code
### Start Ardupilot
From the ardupilot directory
```bash
cd ArduCopter
sim_vehicle.py -v ArduCopter -f octa --model JSON --console # may need run conda deactivate first
```
### Start Gazebo
From python-gazebo directory
```bash
gazebo worlds/test.world --verbose -u
```
### Start Python
From python-gazebo directory  
Open up env/APDemo.ipynb file  
Run blocks to start connection  
### Control Parameters
After all three parts are running, load Octocopter parameters from the ArduPilot console  
There should be an parameters button where you can then choose to load a file  
Choose simOcto.parm in the python-gazebo/env directory  
You will only need to do this once (parameters should save automatically)  
### Starting the drone
In the console where Ardupilot is running, enter these commands after everything loads and connects (look for when a GPS is detected)
```bash
mode guided
arm throttle
takeoff 10
```
The drone should then takeoff in the Gazebo window
### Using a ground control system
To run more advanced flights, you can download a Ground Control system  
IQ_GNC Tutorial: [QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_qgc.md)  
Instead of running commands from terminal, you can set flights from the ground control system instead  
Note: This feature is not tested yet
