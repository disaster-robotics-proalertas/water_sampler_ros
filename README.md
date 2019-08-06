# water-sampler-ros
ROS driver for the Atlas Scientific water sampler

## Description

This ROS package provides a driver for Atlas Scienfitic's [EZO-PMP](https://www.atlas-scientific.com/product_pages/peristaltic/ezo-pmp.html) water dosing pumps (in python), as well as a ROS node for advertising services for operations with the pump, such as check pump state, fill pump, etc.
Specifically, the [PMP driver](https://github.com/disaster-robotics-proalertas/water-sampler-ros/blob/master/scripts/PumpControl.py) implements functions which issue commands to the pumps via I2C (e.g., report dosing volume, fill doser up to X mL), and the [ROS sampler service node](https://github.com/disaster-robotics-proalertas/water-sampler-ros/blob/master/scripts/sampler_srv_node.py) externalizes these functions as ROS services, so any system in the ROS environment can call them.

This package was primarily designed to be used on a Raspberry Pi Zero, controlling four pumps on a Platypus Lutra Prop autonomous boat, for water quality monitoring applications. The hardware and electrical connections' schematic required to use it is illustrated below using [Fritzing](http://fritzing.org/home/).
![Alt text](docs/images/Sampler_fritzing.png?raw=true "Sampler's electric schematic")

We named the four pumps with numbers from 0 to 3, and address them internally in the [PMP driver](https://github.com/disaster-robotics-proalertas/water-sampler-ros/blob/master/scripts/PumpControl.py).
Some modification may be required to use the package with other platforms and/or pump addresses.
Our conventions for naming and addressing the pumps are:

* Pump 0 <--> Address 0x01
* Pump 1 <--> Address 0x02
* Pump 2 <--> Address 0x03
* Pump 3 <--> Address 0x04

When fully assembled, the Sampler should look like the figure below. The four pumps are each connected to a container, with a diameter and height of aproximately 82.6 mm and 96 mm, respectively.
![Alt text](docs/images/Sampler_hardware.jpeg?raw=true "Sampler's hardware")

## Usage

With a roscore running, simply run the sampler_srv_node.py node:

```
rosrun water_sampler_ros sampler_srv_node.py
```

This will advertise the sampler services, which can be seen with the "rosservice list" command.
The services and their arguments are:

* /sampler/fill_pump ----> Args: pump_number mL
* /sampler/check_pump_state ----> Args: pump_number
* /sampler/empty_pump ----> Args: pump_number
* /sampler/stop_pump ----> Args: pump_number

## Installation

* Clone the package to your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/disaster-robotics-proalertas/water-sampler-ros
```

* Make the catkin environment
```
cd ~/catkin_ws/
catkin_make
```

* Source your catkin environment
```
source ~/catkin_ws/devel/setup.bash
```

### Dependencies

* Raspberry Pi + debian-based OS (e.g., [Ubuntu for raspberry with ROS installed](https://downloads.ubiquityrobotics.com/pi.html))
* Robot Operating System for Raspberry Pi[ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Python I2C library for Raspberry Pi: [smbus](https://pypi.org/project/smbus/)

## To-Do:

* Test the package on-site (river or basin) in the autonomous boat
* Advertise the PMP driver functions as actions instead of services (can interrupt, halt or abort actions, with appropriate feedback)

## Collaborators

* [Igor Souza](https://github.com/igorSouzaA): Python PMP Driver
* [Renan Maidana](https://github.com/rgmaidana): ROS Sampler package, sampler service node
* [Guilherme Heck](https://github.com/heckgui): Testing and knowledge of the Lutra boat system
* [Alexandre Amory](https://github.com/amamory): Guidance and logistics
