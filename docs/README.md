# Herbie V.2 Docs

V.1 software is simpler and wheel encoders aren't needed and ROS (Robot Operating System) isn't needed. We made V.2 to use ROS and the wheel encoder output to more carefully match the the spray action to the machine speed. It does this by selecting between three different slices of the camera (preview) image to send to the AI engine. If the machine is going faster, the top slice is used. Medium speed directs that the middle slice is used. And slow speed selects the lower slice of the image. This way the X dimension is more accurate for spraying. The encoder data is published to ROS by an Arduino Mega board as well as a simple speed topic. The Raspi Python script subscribes to this topic. The Mega also controls LEDs that show machine speed and another relay that turns off the pump when machine is stopped, turning sharply, or backing up. This makes the operator's job easier, as manual pump shut down at the end of a pass is not needed.

Both software versions are included in this repo, so you can build whichever is most convenient for you. Or do them both!


# Software Installation

Herbie_v1 doesn't require ROS or the Mega.
Herbie_v2 requires ROS and wheel encoders and the Mega board. It is more accurate for spraying weeds.

The Raspberry Pi OS must be Raspi Buster, 64 bit. Ubuntu is better for ROS but worse for the GPIO pin control and use. And we need GPIO for the pump relay and the wand servo.

Install ROS Noetic, full desktop, and then Rosserial-Python and Pip.
There is a web site with good directions on getting Noetic installed on Buster:
https://varhowto.com/install-ros-noetic-raspberry-pi-4/

I had to install "serial" with pip install serial for the Rosserial Python to work on laptop.

Install DepthAI for the Oak-D camera and AI model. The web site is
https://docs.luxonis.com/en/latest/pages/tutorials/first_steps/#
Best way to install:
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash

On the Raspberry Pi, change directory to user/catkin_ws/src and then clone this repo:
git clone https://github.com/Russ76/Herbie-bot

cd to catkin_ws and run "catkin_make"

Install pigpio for the servo control and edit .bashrc to export the pigpio setting.
export GPIOZERO_PIN_FACTORY=pigpio

For the Arduino Mega, install the Arduino IDE on a laptop. (Or Raspberry Pi)

Install Paul Stoffregren's encoder library, and the Rosserial library on IDE.

Open the .ino code in the IDE and upload it to the Mega board.

Physically install the Raspi into a suitable yard robot with sprayer.

Start Raspi in headless mode.

ssl into Raspi, not as root.


Change directory to "wide-weeds"

Start program with "python3 main_api12.py" (Make a launch file to do all software start, rosserial and weed-detect, and adjust paths so that it works, first time! Will have to be in catkin_ws/src/herbie. Ino code under /firmware) 


# The AI model was developed in this way:

Images were taken from the Oak-D camera, mounted on the yard robot, as it traversed the lawn, using the video1.py script. These images were 1920 x 300 in size. Frames per second were turned down to 15. Those with weeds in them were retained for using in the AI model.

Images were uploaded to Roboflow. Annotation was done on the site, as well as agumenting the collection with flipped images, darkened images, and lightened images. These images were 1920 x 300, in other words, wide and not tall. Big on the X and small on the Y.

We try to retain the wide and short size of these images. This is how the Oak-D camera will send video to the NN model, wide and short, so that the weed spraying wand will be directed left or right only, for it dosn't have fore and aft movement yet. The machine moves forward, of course, and we plan that the speed will allow for good timing and spraying of the weeds. For this reason, only the strongest detection will be sent from Raspi to the servo for moving the wand; there won't be time to spray two spots in a single image.

Export the model with the "YoloV5 Pytorch" format. Don't use "download zip to computer", but the other, "show download code." The link snippit is what will be copied and pasted into the next site.

https://ultralytics.com/

The Yolo5 Ultralytics site Colab Notebook was used for training, and this converted the images into the 1280 x 224 size. (Requires a Google account.) (Only the 1280 size needs to be specified.) This is the maximum width allowed there at this time. Epochs can be turned down from 150 to 100. The training doesn't take long! Paste in the copied Roboflow code snippet at the correct place. (The Yolov5 software needn't be installed locally at all.)

(Sample code snippit:)
!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="xxxxxx")
project = rf.workspace("xxxx").project("full_width_weeds")
dataset = project.version(3).download("yolov5")

After training, save the completed model weights to your local machine. It will be called best.pt. This is what will be loaded into the Luxonis DepthAI conversion web tool, to make the Oak-D (Openvino) blob. Save these files under the Openvino folder on Raspi.

tools.luxonis.com

https://docs.luxonis.com/en/latest/pages/model_conversion/


# Additional robot info

My yardbot is controlled by radio control; it is independent of the Raspi software. The speed lights give indication of the "slice" of image used in the Raspi. When driving, try to keep the middle light lit. If weeds are very heavy (numerous), go slower, into the slowest category. If weeds are light, the machine may go faster. Using the upper slice of camera image will keep the timing correct for the spray wand, in relation to a faster machine speed. My yardbot has an encoder count of 742 per meter. You will probably have to adjust the number of encoder ticks to correspond to your hardware.

The spray wand is installed about 10 inches behind the camera. Camera points straight down and is about 18 inches above lawn. The camera is a fixed focus model. Wand servo is in line with camera height and centered on machine. Servo attaches to wand in center of handle.

A separate 6V battery supplies power for the servo. They can use 5V or 6V but the higher voltage makes the servo respond quicker, which we want. The USB hub provides 5V power for the pump. They are powered by 4.5 volts in the standard consumer battery configuration (3 x 1.5 AAA) but the higher voltage gives a more powerful spray, which is needed since machine doesn't linger over the weed. Adjust the nozzle to the full width spray, not just the narrow stream. The wand is a single elbow type, about 18 inches long. Some herbicide jugs have a two elbow wand which may be too long for Herbie. These gallon herbicide jugs with wand and hose are a great addition to yard care and make the Herbie Robot construction fairly easy. Concentrate is also available, for refilling your empty jug. Also one may use blue dye, for marking weeds sprayed. This evaporates (fades) on the same day.

The spray wand trigger switch has been removed and the spring taken out of the valve, so that herbicide can always flow to pump. (The two halves of the plastic wand shell just pop apart.) Wires have been added so that the external 5V line can provide power to the existing wand pump motor, controlled by the Raspi relay unit. One switch controlled by the RC unit and two relays work in series to enable the pump in normal operation, one relay controlled by Raspi and the other by Mega. The Mega turns off the relay when machine is stopped, reversing, or turning very sharply and the Raspi relay is turned on via weed detection. This control ensures that strong herbicide isn't sprayed where it is not wanted. The "prime" switch bypasses the relays and the radio switch, allowing one to prime the pump and get herbicide flowing through the wand at the start of application. If herbicide won't flow with priming, raise bottle somewhat to facilitate flow. After use, wand should be rinsed with water spraying through it. Turn nozzle tip "off" after rinsing and next use will be easier to prime, as tube will remain filled with liquid.

The Raspi is powered by a battery powerbank. The TP-Link USB hub powers the Oak-D camera and the Mega. Raspi doesn't have enough power output to run the camera by itself. (I believe it will power the Oak-Dlite). My first TP-Link hub had a very small power indicator light, so I added a blue LED to better show power on. This hub is powered by 12V, so works well to wire up to one of the large Yardbot main batteries. 24VDC powers the Yardbot, Sabertooth 2x25 controller, and motors. The motors, gear boxes and wheels are from a "Jazzy" mobility cart, the units purchased through eBay. They are very powerful. I removed the brakes and installed encoders, from US Digital, under the same handy cap on opposite motor end. The Sabertooth is simple to hook up to the RC receiver unit. The Sabertooth is pricey but rock solid and versatile.


Herbie wiring is straight-forward, as follows:
(please refer to images in this folder)


# Preflight checklist - Herbie
-----------------------------
Herbicide bottle full?
Blue marker dye added?
Handheld Radio = ON
Herbie chassis and radio = ON
Drive to and park on lawn.
Herbicide bottletop valve = ON
Herbicide wand nozzle tip = ON
USB hub push button = ON
Check that Blue LED is ON
Prime Pump = ON (until spraying) then OFF
Pump switch on radio = ON
Servo switch = ON
Plug in Raspberry Pi
Turn on laptop computer
Connect to Raspi over ssh
Start up software on Raspi
Is wand moving? Good. Finally,
Drive forward to spray weeds
Ensure spray wand is swinging and spray is spraying
Drive at even speed, straight ahead
Try to keep middle LED on
(Herbie v.1) Turn off pump when turning around at end of pass



# Sample screen output:

No weeds found... 
No weeds found... 
No weeds found... 
No weeds found... 
No weeds found... 
No weeds found... 
No weeds found... 
No weeds found... 
Servo aim=   0.3   Spraying...   Counter=  7770
Servo aim=   -0.6   Spraying...   Counter=  7771
Servo aim=   -0.7   Spraying...   Counter=  7772
Servo aim=   0.2   Spraying...   Counter=  7773
Servo aim=   -0.1   Spraying...   Counter=  7774
Servo aim=   0.2   Spraying...   Counter=  7775
Servo aim=   -0.1   Spraying...   Counter=  7776
 Slow down! Thick weeds! 
Servo aim=   0.1   Spraying...   Counter=  7777
Servo aim=   0.1   Spraying...   Counter=  7778
Servo aim=   0.3   Spraying...   Counter=  7779
Servo aim=   -0.7   Spraying...   Counter=  7780
Servo aim=   0.6   Spraying...   Counter=  7781
No weeds found... 
Servo aim=   -0.3   Spraying...   Counter=  7783
Servo aim=   -0.3   Spraying...   Counter=  7784
No weeds found... 
Servo aim=   -0.5   Spraying...   Counter=  7786
Servo aim=   -0.5   Spraying...   Counter=  7787
Servo aim=   -0.6   Spraying...   Counter=  7788
No weeds found... 
No weeds found... 
No weeds found... 
No weeds found... 

