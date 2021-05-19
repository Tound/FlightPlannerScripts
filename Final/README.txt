-- FlightPlanner Setup and instructions for use --
Created by Thomas Pound
Last updated 19/5/21

This software created by Thomas Pound is available for reuse and modification.
The software is designed to create viable and optimal flight routes for aerial photogrammetry 
surveys using a fixed-wing UAV. The software is built using Python scripts.
The software is cross-platform but some issues may appear on various versions of operating systems.

This folder includes the backend python scripts that can be used with a GUI to create flight paths for
randomly generated terrain.
The scripts also work without a GUI and flight paths can be found for randomly generated terrain.
A 3D view of the flight path is produced as the output

-- Prerequisites --
Python version must be 3+
Python3 must be installed and on the PATH

The following scripts must be in the same directory:
	efficient_flight_path.py
	create_passes.py
	dubins_3D.py
	passes_TSP.py
	Image_Classes.py
	camera_calculations.py
	create_spirals.py

-- Install the required Python packages --
Before being able to use the software the following python packages must be installed.
In the terminal or cmd, the command:

	pip3 install -r requirements.txt

can be used to install all the required python packages.

-- Running of the software --
efficient_flight_path.py is the main file that needs to be ran.
The file can be edited for to input various values for testing 

The command:

	python3 efficient_flight_path.py
		or
	python efficient_flight_path.py

	depending on how your Python is installed on your machine,

can be used to run the scripts.

efficient_flight_path.py can be edited to set the desired variables to create desired flight paths.
