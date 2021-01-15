# MorphDemo
This basic Python GUI was built to allow simple interaction with a morphing octocopter drone simulation I created. A morphing drone is a drone that changes the position of its rotors while flying. The simulator is using and checking the same morphing algorithm that is implemented and working on the physical drones. The purpose of this algorithm, and morphing in general, is to maximize energy efficiency and flight time given an arbitrary payload.

## Installation
The demo requires specific package versions - if you would like to preserve the package versions on your global system, create a virtual environment before using requirements.txt

On your Linux machine, clone this repository using `git clone <repo URL>`. Next, cd into the cloned directory and run the following command.
```
pip install -r requirements.txt
```
When the installation is finished, initialize the GUI by running `python MorphDemo.py`
