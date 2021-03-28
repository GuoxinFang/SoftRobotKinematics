# Soft Robot Kinematics

This is the demo code for soft robot kinematics computing based on geometry computing, which is the implementation of paper:

* "Kinematics of soft robots by geometric computing", Guoxin Fang, Christopher-Denny Matte, Rob B.N. Scharff, Tsz-Ho Kwok, and Charlie C.L. Wang. IEEE Transactions on Robotics, vol.36, no.4, pp.1272-1286, August 2020.

## Installation

Please compline the code with QMake file 'soroKinematics.pro'.

Tested platform: 
* MacOS: QT Creator 
* Windows: Visual Studio + QT-plugin (tested QT version: 5.12.10)

## Usage

Input soft robot models in Open->"soro_body.tet", this is the tetrahedron mesh includes body elements and actuation elements.

Before kinematics computing, press the buttom "pre-process system", it will import the pre-defined chamber selection file installed in "soro_chamber.txt" and "soro_rigid_handle.txt".

Forward Kinematics: define actuator parameter in the spinbox (prefer within the range [1,3.5])

Inverse Kinematics: press buttom "Inverse Kinemtaics".

## Contact

Guoxin Fang： G.Fang-1@tudelft.nl
