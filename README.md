# Docker Solution for HCR

#### A) Installation
Preparation: 
- Controller computer (to control the HCR robot via RVIZ)
    - has graphical interface
    - has Ubuntu 16.04 or 18.04
    - has ROS Kinetic Desktop installed.
    - has Wifi connection
- Driver computer (to run HCR robot drivers)
    - has x86_64 architecture
    - has ubuntu 16.04 or 18.04 installed
    - has Docker Engine Community 19.x.x or above 
    - has Wifi connection
    - has at least 4GB of RAM

- To install: 
    - On both controller computer and driver computer download and extract or clone the hcrdocker project from ...
    - On driver computer excute "./build" to build the HCR docker image and store it locally.

#### B) Usage:
- Start up:
    - On driver computer:
        - execute "./hcr start" to start up the hcr drivers.
        - you can do "./hcr help" to see available options.
    - On controller computer:
        - execute "./control \<ip address of controller computer\> \<option\>"
        - you can do "./control help" to see what options is available.


- Rebuild: execute "./build" on driver computer to rebuild the HCR docker image incase you have change the HCR code base.




