# Soft Robot Kinematics by Geometry Computing

This is the demo code for soft robot kinematics computing based on geometry computing, which is the implementation of paper:

* "Kinematics of soft robots by geometric computing", Guoxin Fang, Christopher-Denny Matte, Rob B.N. Scharff, Tsz-Ho Kwok, and Charlie C.L. Wang. IEEE Transactions on Robotics, vol.36, no.4, pp.1272-1286, August 2020. (https://ieeexplore.ieee.org/document/9082704).

## Installation

Please compline the code with QMake file 'soroKinematics.pro'.

Tested platform: 
* MacOS: QT Creator 
* Windows: Visual Studio + QT-plugin (tested QT version: 5.12.10)
  (Remark: Please enable OpenMP acceleration in Visual Studio environment)

## Usage

Input soft robot models in File->Open->"soro_body.tet", this is the tetrahedron mesh includes body elements and actuation elements.

Before kinematics computing, press the buttom "pre-process system", it will import the pre-defined chamber selection file installed in "soro_chamber.txt" and "soro_rigid_handle.txt".

Run Forward Kinematics: define actuator parameter in the spinbox (prefer within the range [1,3.5]) and press 'forward kinematics' buttom.

Run Inverse Kinematics: press buttom "Inverse Kinemtaics". Notice that IK computing by numerical-based simulation can be slow, please check our recent work for real-time IK computing

* "Jacobian-based learning for inverse kinematics of soft robots", Guoxin Fang, Yingjun Tian, Zhi-Xin Yang, Jo MP Geraedts, Charlie CL Wang, (https://arxiv.org/abs/2012.13965).

## Contact

Guoxin Fang： G.Fang-1@tudelft.nl
