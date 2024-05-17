# Ganbei
## Inexpensive Mobile Manipulator

This is a small, cheap robot with verbal interaction, symbolic learning, an arm, and 3D sensing (soon). It is a lightly modified version of the [Master Pi](https://www.hiwonder.com/products/masterpi?variant=39783006994519) available from Hiwonder. The robot uses the [ALIA](https://github.com/jconnell11/ALIA) cognitive library along with Python support code to run on a Raspberry Pi 4b. Check out this [video](https://youtu.be/qWLANb0PmbM) of some simple reasoning.

![Herbie robot](Herbie.jpg)

### Getting Started

There are a number of [software](doc/software.md) changes that should be performed first. After this, the robot needs a few [hardware](doc/hardware.md) modifications including the installation of a sound card and IMU. The basic robot with Pi 4 processor costs about $300, while the extensions add about $50.

When all the modifications are complete, you should be able to invoke the demo by a short press of the __front__ button (approach from above) on the top circuitboard. Use a short press of the back button (approach under shell) to exit. A long press of the back button will cleanly shut down the robot, but you must still power it off manually. Note, you will need to say "robot" or the network name of the machine (e.g. "Herbie") to get the robot's attention (eyes turn green). 

You can also type to the robot if you prefer that to speaking. The easiest way is to connect via RealVNC and type the command "demo" (or alternatively "cd ~/Ganbei" then "sudo python3 Ganbei_act.py"). If you instead started the robot using the button, you can use the command "connect" (e.g. from a remote SSH session). No attention word is necessary when typing.

Right now the robot can move on command (e.g. "drive forward" or "gaze left" or "shift up"). In general, it can perform all the sorts of learning shown in this [video](https://youtu.be/EjzdjWy3SKM). However, it cannot navigate, or find and grab objects yet. This will be possible eventually ...

---

May 2024 - Jonathan Connell - jconnell@alum.mit.edu


