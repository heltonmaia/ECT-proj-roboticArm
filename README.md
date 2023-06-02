# Robotic Arm Control Using Computer Vision and Machine Learning
Project developed at the Robotics and Automation Laboratory (UFRN/LAR)

Computer vision, 3D printing, and Arduino integration project, with the main objective of enabling a robotic arm to reproduce the movements of a human arm. The project involves a series of technologies using the OpenCV computer vision library, such as real-time capture of images through a webcam or smartphone. Then, the detection of points of interest, such as the hand and arm, is performed using the MediaPipe framework. Finally, the control of the robotic arm, which was made via 3D printing, is performed by Arduino according to serial port commands given by the python script using the Pyserial module. This control is based on an intelligent decision-making system that links the robotic arm's movements to the user's.

This project was developed and tested on **Windows 10 and 11**, using **python 3.10.9 within the Anaconda package** and the **Arduino IDE 2.0**

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
pip istall opencv-python
```
```
pip istall mediapipe
```
```
pip install pyserial
```
```
pip install ipykernel
```
After all the installations are complete, you need to register your conda virtual environment using the ipykernel module. If you don't do this, the Jupyter Notebook will not recognize your virtual environment and the packages on it. Once again change MyEnvironmentName to the name you gave it
```
python -m ipykernel install --user --name=MyEnvironmentName
```
### **Detecting the hand**
To use the main algorithm, it is necessary to ensure correct hand detection of the hand via mediapipe.
#### **Accessing the camera**
If you don't have a webcam, you can use the Droidcam app on a smartphone to use the phone's camera as a webcam
  - Install the Droidcam app (Android or IOS): [Download page](https://www.dev47apps.com)
  - Install the Droidcam client on Windows: [Download page](https://www.dev47apps.com/droidcam/windows/)
  - Then follow the instructions to connect: [Connection guide](https://www.dev47apps.com/droidcam/connect/)

Once you have access to a camera, open Jupyter notebook 
