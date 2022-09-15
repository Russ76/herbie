# Herbie_Bot will spray the weeds in your lawn and not spray the grass! 

These gallon jugs of lawn-safe herbicide with a battery-powered spray wand are now readily available and they do a good job. However, after spraying manually for an hour with my hand and arm starting to cramp up, I realized that my Yard_Bot (mowbot/snowbot/materialbot) could do the spraying.

Many kinds of larger, yard robots or carts could do the carrying and moving. The AI engine finds the weeds and overlooks the grass.

The spray wand mounts to a servo to move it left/right. Herbie uses the OpenCV Oak-D Camera and AI to detect the weeds and only spray them. The spray wand trigger is removed and the valve set always open. The small batteries are removed and the wand is wired to other power and controlled by the software. A Raspberry Pi 4 with 4 Gig RAM works well to run things and interact with the Camera.

The servo is directed to one of about 15 positions in the Y dimension by the Raspi as the AI detects weeds amongst the grass. The Movidius chip in the Oak camera does the object detection routine very quickly. The wand is only about 12 inches behind the camera, and it sprayes those pesky weeds!

The fore/aft (X) dimension is not addressed in Herbie_Bot v.1, the operator must watch the speed of the robot on the lawn and coordinate the spraying aim. It isn't difficult to do, a steady, slow speed works well. 
