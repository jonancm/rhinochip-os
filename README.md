# RhinoChip OS

RhinoChip OS is the firmware of the RhinoChip Controller for Educational Robotics. The RhinoChip platform provides a controller for the educational robots Rhino XR-4 and Rhino SCARA which is intended to replace the Rhino Mark IV controller. RhinoChip OS is the software that powers the RhinoChip platform.

RhinoChip OS consists of two programs: the general purpose core (`gpcore`) and the motor control program (`motorctl`). The `gpcore` is the program that runs on the dsPIC30F4013 General Purpose Microcontroller and communicates with the host PC. `motorctl` is the program that runs on the dsPIC30F4011 Motor Control Microcontroller and drives the motors of the robotic arm.
