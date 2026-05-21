# Credentials & Calibration

## Microsoft Azure Credentials

The system is default coded to use Microsoft Azure speech recognition, which is essentially __free__ for low intensity usage. However, you will need credentials to access this on-line service. Start by signing up [here](https://portal.azure.com/#create/Microsoft.CognitiveServicesSpeechServices) (possibly making a Microsoft account first) then select "Speech Services" and "+ Create". Finally, click "Manage keys" and modify local configuration file [Herbie_azure](../config/Herbie_azure.yaml) (or whatever your robot's name is) with valid "Key" and "Location" strings. This only needs to be done once.

## Arm Calibration

The midpoint and scaling of the arm servos can vary from one unit to another. Getting accurate values for these parameters is crucial for gaze control and grasping objects. From the Ganbei/scripts directory run the command below and follow its instructions to move the arm into various poses. 

    python mpi_arm_cal.py

To further refine the calibration, follow up by running the related command below. This tries to optimize the arm pose for a typical grasping location.

    python mpi_grab_cal.py

Both these utilities will automatically write new values to the configuration file [Herbie_servo](../config/Herbie_servo.yaml) (or whatever your robot's name is). 

## TOF Calibration

This is needed for proper hand-eye coordination and is controlled by the [Herbie_cam](../config/Herbie_cam.cal) file (or whatever your robot's name is). The file has a single line beginning with "grok_cal" whose fields are as shown below. The first 3 values are for the time-of-flight sensor while the second 3 values are for the color camera. You must edit this file manually.

    grok_cal  tof_pan tof_tilt tof_roll  cam_pan cam_tilt cam_roll

To calibrate the TOF sensor, first place the robot in environment where it has a large expanse of floor in front of it. Then fire up the complete system in debug mode using the command below, and enter the ALIA request shown. Now manually copy the displayed "dr" value to the third number in the calibration file. This corresponds to the __roll__ of the sensor.

    demo 3
    > look slightly down

The next calibration step needs to be repeated any time the arm servo parameters (above) get changed. Place a small block directly in front of the robot so that its _center_ is 4" (10cm) in front of the bumper. Run the command below and enter the subsequent ALIA request. You want the displayed object position to read (0.0 7.7) within +/- 0.2 inches. 

    demo 4
    > look far down

Doing this requires editing more configuration values. The first number after "grok_cal" affects the __pan__ of the sensor – a higher value here makes the displayed x coordinate _smaller_. Similarly, the second value changes the __tilt__ compensation – a higher value for this parameter makes the y coordinate _larger_.  Hit ESC to exit ALIA, change the values, then re-run the procedure until the reported object position becomes reasonably accurate. 

## Camera Calibration

Finally, you need to get the range-finder and color camera to agree on where objects are. If you change the TOF calibration (above) you will likely need to redo this color calibration. Invoke demo mode and enter the request given below. The right hand image will now show a mask where the portion of the color image corresponding to the nearest object shows through. 

    demo 6
    > look down

Place an elongated object in front of the robot and adjust the camera's __roll__ until the mask is at the same angle as the object (the alignment is likely to be far off). As before, edit the configuration file [Herbie_cam](../config/Herbie_cam.cal) fiddling with the final value in the line, then restart the program. Keep trying different values (higher numbers rotate the mask _clockwise_) until the mask and the object are parallel.

Next, fix up the __pan__ and __tilt__ compensation values. For this it is often easier to look at the object bounding box in the left hand image. Increasing the fourth value in the line moves the box _right_, while increasing the fifth value moves the box _down_. Again, iterate between running the system and editing the file until the box is nicely centered on the object. At this point the mask on the right hand side should show only the object, no table fragments around it.

---

May 2026 - Jonathan Connell - jconnell@alum.mit.edu