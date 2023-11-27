Project developed at the Robotics and Automation Laboratory (UFRN/LAR)

Current project members:
- [Kennymar Bezera de Oliveira](https://github.com/KennymarOliveira)
- [Thiago Vinicius Cardoso Lopes]
<!---
- [Geraldo Almeida Neto](https://github.com/geraldoAn)
- [Franklin Thiago Pereira Correia](https://github.com/FranklynThP)
-->

Supervisor: [Helton Maia](https://github.com/heltonmaia/) 

# Robotic Arm Control Using Computer Vision and Machine Learning

Computer vision, 3D printing, and Arduino integration project, with the main objective of enabling a robotic arm to reproduce the movements of a human arm. The project involves a series of technologies using the OpenCV computer vision library, such as real-time capture of images through a webcam or smartphone. Then, the detection of points of interest, such as the hand and arm, is performed using the MediaPipe framework. Finally, the control of the robotic arm, which was made via 3D printing, is performed by Arduino according to serial port commands given by the python script using the Pyserial module. This control is based on an intelligent decision-making system that links the robotic arm's movements to the user's.

This project was developed and tested on **Windows 10 and 11**, using **python 3.10.9 within the Anaconda package** and the **Arduino IDE 2.0**

https://github.com/heltonmaia/ECT-proj-roboticArm/assets/4681481/2796c126-4182-4c66-be8b-e16110343908

Project video 

# Using the hand controller algorithm
### **Requirements**
  - Anaconda package: [Install Guide](https://docs.anaconda.com/free/navigator/install/)
  - OpenCV
  - Mediapipe
  - Pyserial
  - ipykernel
  - Webcam or a smartphone

### **Creating a python virtual environment**
In this part, the basic configuration necessary for the implementation of the algorithm is described according to the specified requirements.

Assuming that the installation of the Anaconda package has already been done, it is time to create a virtual environment, which will provide the installation of only the necessary modules and an independent environment for the development and execution of the project using specific versions according to the need.

After opening the Anaconda prompt, follow the instructions changing MyEnvironmentName to the desired name for your environment
```
conda create --name MyEnvironmentName python=3.10
```
With the environment now created, activate it changing MyEnvironmentName to the name you gave it
```
conda activate MyEnvironmentName
```
### **Installing modules and registering the environment**
Now the installation of the modules listed in the requirements is perfomed, and they will only be installed in your virtual environment, which is activated
```
pip install opencv-python
```
```
pip install mediapipe
```
```
pip install pyserial
```
```
pip install ipykernel
```
After all the installations are complete, you need to register your conda virtual environment using the ipykernel module. If you don't do this, the Jupyter Notebook, that was installed within the Anaconda package, will not recognize your virtual environment and the modules on it. Once again change MyEnvironmentName to the name you gave it
```
python -m ipykernel install --user --name=MyEnvironmentName
```
### **Detecting the hand**
To use the main algorithm, it is necessary to ensure the correct hand detection via mediapipe.
#### **Accessing the camera**
If you don't have a webcam, you can use the Droidcam app on a smartphone to use the phone's camera as a webcam
  - Install the Droidcam app (Android or IOS): [Download page](https://www.dev47apps.com)
  - Install the Droidcam client on Windows: [Download page](https://www.dev47apps.com/droidcam/windows/)
  - Then follow the instructions to connect: [Connection guide](https://www.dev47apps.com/droidcam/connect/)

Once you have access to a camera, use Jupyter Notebook to open the file "right hand detection in range.py" available in this project, then change the kernel within the code editor to your virtual environment's one. Now you can run the script and see if the right hand is detected within the specified range.

<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/hand%20detection%20in%20range.PNG" style="width: 600px; height: 420px;"></p>

After the correct detection of the hand it is necessary to close the jupyter notebook and deactivate the virtual environment, and only after the Arduino part is done be activated again, otherwise both codes will try to take control of the serial port and the serial commands comming from the python script will not be read. The next access to the Anaconda prompt and the Jupyter Notebook should be via the Anaconda Navigator, after running it as administrator and while the Arduino script is running.

You can deactivate your virtual environment typing in the Anaconda prompt
```
conda deactivate
```

### **Configuring the Arduino IDE**
Once you installed the Arduino IDE, you can download zip file the VarSpeedServo library available [here](https://github.com/netlabtoolkit/VarSpeedServo) and install it by following [this guide](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries).
Once complete, copy the code of the file "serial port command receiver.cpp" to the Arduino IDE, and start the next part. 

### **Assembling the circuit**
The following circuit was simulated using the [Wokwi](https://wokwi.com/projects/new/arduino-uno) online platform, and considering that the four servomotors are already fitted to the robotic arm, the circuit of this image bellow must be physically assembled.

<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/wokwi.PNG" style="width: 600px; height: 420px;"></p>

Make sure that the digital pins on the Arduino board receiving the signal jumpers from the servomotors are the integer variable values specified in the code "serial port command receiver.cpp" as "something_pin". 

Select the right serial port within the IDE (the one that says "Arduino Uno"), compile and send the code to the board.

### **Using the main algorithm**
While the Arduino script is running, run Anaconda Navigator as administrator and launch the Anaconda prompt. Activate your virtual environment, go back to the Navigator interface and launch Jupyter Notebook.

Use the code of the main algorithm named "right hand controller using serial commands.py" to a Jupyter Notebook file. 

Locate the declaration of the object "ser", which uses the pyserial module
``` python
ser = serial.Serial("COMX", 9600, timeout=0.01)
```
Change "COMX" to the serial port currently connected to the arduino(check the Arduino IDE in case you don't remember), for example COM3, COM12 etc.

Now you can run the the code and controll the robotic arm by the movements of your right hand. Check the examples bellow to understand the expected hand positions.

# **Examples**
The following images are examples that show what approximate positions the right hand must be in for detections to be made correctly.
<h3 align="center"><b>Open hand</b></h3>
<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/open%20hand.PNG" style="width: 600px; height: 420px;"></p>

<h3 align="center"><b>Closed hand</b></h3>
<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/closed%20hand.PNG" style="width: 600px; height: 420px;"></p>
