# Robot Operation

## Buttons

First, turn the robot on using the slide switch accessible through the slot in the back shell. The robot will "whistle" when it has finished booting. 

Now you can invoke the standard demo by a short press of the __front__ button on the top circuitboard (approach from above) . You will hear two short beeps then the robot will start speaking when the program has finished initializing. If instead you hear an indignant "squawk", it means the program has crashed for some reason! Use a short press of the __back__ button (approach under shell) to exit the demo. 

Finally, use a __long press__ of the _front_ button to initiate a clean shut down of the robot. Wait until the small green light buried in front of the power switch stops blinking, then finish by powering off manually with the slide switch.

## Interacting

When the robot is running you should be able to speak directly to it. In some cases you may need to say "robot" or the network name of the machine (e.g. "Herbie") to get the robot's __attention__. You can also preface whatever you want to say with this wake-up word, such as "robot, grab the object". If its eyes are already green you can skip any wake-up cue (i.e. just say "grab the object").

You can ask general questions, tell the robot to perform certain motions, ask it about visual objects, and teach new procedures. Some examples are given below:

<small>

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

</small>

## "Demo" Command

You can also type to the robot if you prefer that to speaking. The easiest way is to connect via RealVNC and type the command "demo" (or alternatively "cd ~/Ganbei" then "python Ganbei_vis.py"). The conversation line includes very limited editing such as backspace, up arrow (recall previous), and delete (start over). It will autocorrect some mis-typings (after ENTER is pressed). No attention word is necessary when typing.

If the "demo" command is given a numeric argument it can display internal images. The left hand panel is always the dewarped color image with important objects marked, while the right hand panel depends on the argument given.

<small>

     0 = none
     1 = integrated navigation overhead map
     2 = front-facing false color depth 

     3 = object deviations from flat
     4 = object overhead height map
     5 = forward range with object boxes
     6 = object mask, dims, and colors
     7 = table location for object deposit

     8 = current near floor heights 
     9 = deviations from planar floor
    10 = current obstacle classification
    11 = traversable regions + sensors

    12 = person overhead height map 
    13 = potential people over min height 
    14 = person head and shoulders
    15 = person gaze direction

    16 = sound direction wrt heads
    17 = visual motion regions

</small>

## Remote Typing

If you start the robot using the front button, you can connect to it later using SSH (e.g. from your phone). Once you are logged in, give the command "connect" to bring up the ongoing conversation line. The robot will accept either spoken or typed commands in this mode.

---

May 2026 - Jonathan Connell - jconnell@alum.mit.edu
