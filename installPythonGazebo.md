# Gazebo Testbed for Octorotor Control Algorithms

This is the Gazebo framework for testing and training of RL Flight Controllers in a simulated environment in the Gazebo simulator.

The testbed front-end is a python interface (see `env/`). The backend is either the Gazebo simulation environment (see `worlds/`, `plugins/`, `models/`) or a python simulation (see `env/multirotor`).


## Installation

The installation process requires several steps:

To run the gazebo physics simulation (host):

1. Building and installing Gazebo,
2. Building plugins for Gazebo simulation,

On the machine using the python interface:

1. Python virtual environment,

If development is being done on a (remote) machine using gazebo GUI over ssh:

1. X-Display server


### Installing Gazebo

A shell script file called `build_gazebo.sh` is included in the repository. This shell script installs all of the required dependencies for DART version 6.7.0 and Gazebo version 11.5.1. DART is a physics engine that can be implemented in Gazebo in place of the default ODE physics engine and is highly recommended over the default engine. In order to use DART with Gazebo, Gazebo must be installed from source and the machine must build and install DART prior to installing Gazebo.

The `build_gazebo.sh` script handles the building and installing of Gazebo and DART. Building Gazebo from source is very resource intensive. Running the script may take more than an hour to execute.

To install, execute:

	git clone -b gym-env --recurse-submodules https://github.com/allanwzhang/python-gazebo.git
	cd gazebo-testbed
	sudo MAKE_FLAGS=-j4 ./build_gazebo.sh
	

### Building Required Plugins

Once Gazebo is installed, the plugins for this specific simulation can be build.

Gazebo uses pre-compiled libraries called plugins to do programmatically run simulations. The Testbed uses several plugins to simulate octorotor digital twins and communicate envrionment states back to python code.

To build all of the required plugins, execute:

	cd plugins
	./build.sh

The included plugins are as follows in the gazebo/plugins folder:

`Tarot_Motor_Model`

- These are *model* plugins. Both are attached directly onto octorotor digital twins via a line written in the `models/tarot_t18/model.sdf` file.
- These plugins are responsible for subscribing to motor commands and applying forces and torques based on the models derived from the Tarot T18 octorotor.
					
`Octorotor_Env`

- This is a *world* plugin. It can be attached to any saved Gazebo world by altering the `.world` file.
- This plugin is responsible for forcing the simulation to be paused on startup and stepping the simulation manually upon receiving UDP packets containing the action messages to be applied to the the octocopter. UDP packets can be sent via the python octorotor controller in the `env/` folder.

Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```
source /path/to/python-gazebo/setup.sh
```

Save and close the text editor.

Reload `~/.bashrc`:
```
. ~/.bashrc
```

### Python virtual environment

Install `conda` (the full of minified version) from the [website](https://docs.conda.io/en/latest/miniconda.html).

Install the python virtual environment:

```bash
conda env create --file ./environment.yml
conda activate gazebo
# python work here
```

The default environment name is `gazebo`, but it can be overwritten using the `-n NAME` flag.

### X-display server

On windows, install the [`VcXsrV` application](https://sourceforge.net/projects/vcxsrv/).

If using the gazebo back-end with its GUI, run the application, setting:

1. Display port number to 0,
2. Unchecking "Native opengl" setting

For further troubleshooting:
Run this in your windows terminal
```bash
setx DISPLAY "127.0.0.1:0.0"
```
Make sure to have these permissions to your ~/.ssh/config file (if connecting through ssh)  
```
ForwardAgent yes  
ForwardX11Trusted yes  
ForwardX11 yes  
```
Installation complete! Go back to initial [readme](https://github.com/allanwzhang/python-gazebo).
