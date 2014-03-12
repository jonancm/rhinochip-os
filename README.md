# RhinoChip OS

RhinoChip OS is the firmware of the RhinoChip Controller for Educational Robotics.

The RhinoChip platform provides a controller for the educational robots Rhino XR-4 and Rhino SCARA and is intended to replace the Rhino Mark IV controller. RhinoChip OS is the software that powers the RhinoChip platform.

RhinoChip OS consists of two programs: the *general purpose core* (`gpcore`) and the *motor control program* (`motorctl`).

The `gpcore` is the program that runs on the *General Purpose Microcontroller Unit* (GPMCU) and communicates with the host PC over the RS-232C protocol in order to interpret the commands sent from the host PC and coordinate the actions and movements indicated by those commands, which are excecuted by `motorctl`.

`motorctl` is the program that runs on the *Motor Control Microcontroller* (MCMCU) and drives the motors of the robotic arm, executing the movement commands indicated by the `gpcore`.
