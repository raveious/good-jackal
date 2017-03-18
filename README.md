# Good Little Jackal

- Teams should develop a URDF model for the LMS200/291. You may assume the mass of the LIDAR to be uniform.
- Add the URDF model to the Jackal Description.
- Define an appropriate TF in which to publish 'laser scan' data from the SICK.
- Use the SICK Toolbox wrapper to publish LIDAR data in this reference frame.
- Demonstrate operation of the Jackal and LIDAR using rviz.
- You should assume you will have no Wi-Fi connectivity. Start the Jackal moving randomly or following some pattern for a predefined time.
- Document your approach, use of the package, and describe your strategy on the github wiki page.
- Present a map of a floor on the EERC. Include this map on your github wiki page.
- Demonstrate operation of your mapping program at a TBD location after presentations on March 1.

## Local Development

After cloning this repository, follow these steps to setup the development environment
```
$ cd lab_6_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ source devel/setup.bash
```
