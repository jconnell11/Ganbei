# Robot Operation

## Buttons

First, turn the robot on using the slide switch accessible through the slot in the back shell. The robot will "whistle" when it has finished booting. 

Now you can invoke the standard demo by a short press of the __front__ button (approach from above) on the top circuitboard. The robot will start speaking when the program is finished initializing. If you hear an indignant "squawk" it means the program has crashed for some reason.

Use a short press of the __back__ button (approach under shell) to exit. A _long press_ of the back button will initiate a clean shut down of the robot. You must still power off manually, but wait until the small green light in front of the power switch stops blinking. 

## Interacting

When the robot is running you should be able to speak directly to it. Note, you may need to say "robot" or the network name of the machine (e.g. "Herbie") to get the robot's __attention__. You can also preface whatever you want to say with this wake-up word, such as "robot, grab the object". If its eyes are already green you can skip any wake-up cue (i.e. just say "grab the object").

You can ask general questions, tell the robot to perform certain motions, ask it about visual objects, and teach new procedures. Some examples are given below:

    what is your name?
    what am I?
    step forward
    turn left 30 degrees
    open the hand
    extend your arm
    lift the short red object
    put it down
    put the tall object on the long object
    move the smallest object to the left of the blue object
    where is the nearest object? 
    how many white objects are there?
    is the skinny object bigger than the blue object?
    to cha-cha drive forward then drive backwards
    don't grab big things but instead say yuck
    before grabbing an object open the hand then close the hand
    to dance spin around and also say la la la la la wee
    keep looking left while wandering
    are you wandering?
    stop wandering
    did you wander?

## "Demo" Command

You can also type to the robot if you prefer that to speaking. The easiest way is to connect via RealVNC and type the command "demo" (or alternatively "cd ~/Ganbei" then "sudo python3 Ganbei_vis.py"). The conversation line includes very limited editing such as backspace, up arrow (recall previous), and delete (start over). It will autocorrect some mis-typing (after ENTER pressed). No attention word is necessary when typing.

If the "demo" command is given a numeric argument it can display internal images. The left hand panel is always the dewarped color image with important objects marked while the right hand panel depends on the argument given.

    1 = overhead navigation map with obstacles and floor areas
    2 = forward looking depth image (bright = close)
    3 = overhead view of surface and estimated pose
    4 = detected objects and coordinates of closest thing
    5 = object mask and inferred color for closest thing
    6 = free surface areas and selected deposit location

## Remote Typing

If you start the robot using the front button, you can connect to it later using SSH (e.g. from your phone). Once you are logged in, type the command "connect" to bring up the ongoing conversation line.

---

July 2025 - Jonathan Connell - jconnell@alum.mit.edu
