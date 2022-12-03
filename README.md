# Shaastra 2023

Packages for Shastra 2023 PS on autonomous drone delivery.
Check out planning [here](https://docs.google.com/spreadsheets/d/19zKDOTPgZpVa8UDxUA69clECNt7pycsukBhtuKTbgz0/edit?usp=sharing)

---

## Installation

Add this in your .bashrc

```bash
source /usr/share/gazebo/setup.sh # put in .bashrc
```

## Setup

- Clone this repo in a new workspace as below.  
  Don't build this workspace now.

```bash
mkdir -p shastra_ws/src && cd shastra_ws/
catkin init
cd src/
git clone https://github.com/AerialRobotics-IITK/rotors_simulator .
git submodule init
git submodule update
```

- Follow the installation steps [here], to setup RotorS fork in a seperate workspace.

> NOTE: If you have `rotorS` package of our fork, [RotorS_ARIITK](https://github.com/AerialRobotics-IITK/rotors_simulator) in another workspace, then you don't need to follow along with the steps below.

- Keeping the above `rotorS` sourced, check via `echo $ROS_PACKAGE_PATH | tr ':' '\n'`, build this package now. This will make sure that every time you source this workspace, `rotorS` gets sourced as well.

---

### Problems

- GAZEBO_PLUGIN_PATH inside a package points to where exactly in the package, knowing that it is needed to point at `.so` library files
