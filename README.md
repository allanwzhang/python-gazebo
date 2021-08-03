

**Gazebo Testbed for Octorotor Control Algorithms (Version 0)**

This is the Gazebo framework for testing and training of RL Flight Controllers in a simulated 
environment in the Gazebo simulator.


**Installation**

A shell script file called "build_gazebo.sh" is included in the repository. This shell script installs all of the required dependencies for DART version 6.7.0 and Gazebo version 11.5.1. DART is a physics engine that can be implemented in Gazebo in place of the default ODE physics engine and is highly recommended over the default engine. In order to use DART with Gazebo, Gazebo must be installed from source and the machine must build and install DART prior to installing Gazebo.

The build_gazebo.sh script handles the building and installing of Gazebo and DART. Building Gazebo from source is very resource intensive. Running the script may take more than an hour to execute.

To install, execute:

	git clone https://git.isis.vanderbilt.edu/systemwidesafety/gazebo-testbed.git
	
	cd gazebo-testbed
	
	sudo MAKE_FLAGS=-j4 ./build_gazebo.sh
	

**Building Required Plugins**

Gazebo uses pre-compiled libraries called plugins to do programmatically run simulations. The Testbed uses several plugins to simulate octorotor digital twins and communicate envrionment states back to python code.

To build all of the required plugins, execute:

	cd gazebo/plugins
	
	./build.sh

The included plugins are as follows in the gazebo/plugins folder:

	* Tarot_Motor_Model - This is a *model* plugin. It is attached directly onto octorotor digital 								twins via a line written in the "model.sdf" file related to the specific 								digital twin.
	
						- This plugin is responsible for subscribing to motor commands and applying 							forces and torques based on the models derived from the Tarot T18 								octorotor. Using this plugin on any digital twin other than the Tarot T18 								will lead to inaccurate forces and torques calculation.
						
						- There are two versions. "Tarot_Motor_Model_Old" is modeled after an 								incorrect version of the Tarot T18. This plugin is related to the 								tarot_old digital twin.
						
							"Tarot_Motor_Model" is the most recent motor model, attached to the 							tarot_t18 digital twin. This model features the corect "spider-like" 								motor orientation of the Tarot T18, with motors 1 and 8 on the right and 								left side of the front, respectively.
							
	* Tarot_Env_Plugin	- This is a *world* plugin. It can be attached to any saved Gazebo world by 							altering the .world file. See sim_env.world in gazebo/worlds. Note that 							the .world file contains the tarot_env_plugin and a tag specifying that 							tarot_t18 model is in this world.
	
						- This plugin is responsible for forcing the simulation to be paused on 							startup and stepping the simulation manually upon receiving UDP packets 							containing the RPM values to be applied to the 8 motors of the 								octocopter. UDP packets can be sent via the python octorotor controller 							in the env folder.
						
	*	Tarot_PB		- This plugin defines the custom Google Protobuff message that the motor 								model listens to for motor commands.
	
						- The custom message is an 8 Dimensional Vector of doubles representing the 							RPM values of motors 1 - 8 respectively.
						



**Using the Environment**

Will explain this in the future.


