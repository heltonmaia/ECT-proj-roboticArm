# Robotic Arm Control Using Computer Vision and Machine Learning

This project encompasses computer vision, machine learning, 3D printing and integrates with Arduino, with the main objective of enabling robotic arms to reproduce the movements of a human arm. The project involves a series of technologies using the OpenCV computer vision library, such as capturing images in real time using a webcam or smartphone. Then, detecting relevant information from the user's hand through our neural network trained based on the YOLOv8 architecture. Finally, the control of the robotic arm, which was done via 3D printing, is carried out by Arduino according to serial port commands given by the python script using the Pyserial module. This control is based on an intelligent decision-making system that links the user's hand movements to the robotic arm.

[Project video](https://github.com/heltonmaia/ECT-proj-roboticArm/assets/4681481/2796c126-4182-4c66-be8b-e16110343908)

# How to use
### **Requirements**
  - Anaconda Distribution ([Install Guide](https://docs.anaconda.com/free/anaconda/install/windows/)) or any IDE of your choice
  - Python 3.10 ([Install Guide](https://www.python.org/downloads/release/python-3100/)) (If you choose to use an IDE of your choice)
  - OpenCV
  - Ultralytics
  - Pyserial
  - ipykernel
  - Webcam or a smartphone

### **Creating a virtual environment**
In this part, the basic configuration necessary for the implementation of the algorithm is described according to the specified requirements.

Assuming that the installation of the Anaconda package (or your favorite IDE) has already been done, it is time to create a virtual environment, which will provide the installation of only the necessary modules and an independent environment for the development and execution of the project using specific versions according to the need.

If you are using an IDE of your choice, just make sure to set up your virtual environment using python 3.10 version. In this tutorial, we will just show how to use the Conda virtual environment manager, which is already included in the Anaconda package.

Open the Anaconda prompt and follow the command changing MyEnvironmentName to the desired name for your environment.
```
conda create --name MyEnvironmentName python=3.10
```
With the environment now created, activate it changing MyEnvironmentName to the name you gave it.
```
conda activate MyEnvironmentName
```
### **Installing modules and registering the environment**
Here we need to install the necessary packages listed in the requirements section. Keep in mind that they will only be installed in your virtual environment, which is currently activated.

This part also applies if you are using the IDE of your choice and another virtual environment manager. 

Now install the packages one by one using the following commands.
```
pip install opencv-python
```
```
pip install ultralytics
```
```
pip install pyserial
```
```
pip install ipykernel
```
In case you are using the Anaconda package, you need to register your conda virtual environment using the ipykernel module. If you don't do this, the Jupyter Notebook, that was installed within the Anaconda package, will not recognize your virtual environment and the modules on it. Once again change MyEnvironmentName to the name you gave it
```
python -m ipykernel install --user --name=MyEnvironmentName
```
### **Accessing the camera**
If you don't have a webcam, you can use the Droidcam app on a smartphone to use the phone's camera as a webcam

  - Install the Droidcam app (Android or IOS): [Download Page](https://www.dev47apps.com)
  - Install the Droidcam client on Windows: [Download Page](https://www.dev47apps.com/droidcam/windows/)
  - Then follow the instructions to connect: [Connection Guide](https://www.dev47apps.com/droidcam/connect/)

### **Detecting the hand**
To use the main algorithm, it is necessary to ensure the correct hand detection using our neural network.

Once you have access to a camera, use Jupyter Notebook to open the file "test-hand-detection.py" available in this project. 

Look for the "Kernel" tab above the code within Jupyter Notebook editor, and then change the kernel to your virtual environment's one.

Download our "weight-hand-segmentation-v11.pt" file, available in the weights folder of this project, and put it in the same directory as the "test-hand-detection.py" script. 

With our weights file, you will be able to use our trained neural network to detect and segment any hand in the camera feed.

Now you can run the script and see if the hand is detected and segmented correctly as shown below.

<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/hand%20detection%20in%20range.png?raw=true" style="width: 600px; height: 420px;"></p>

After the correct detection of the hand it is necessary to close the jupyter notebook (or the IDE that you are using) and deactivate the virtual environment, and only after the Arduino part is done be activated again, otherwise both codes will try to take control of the serial port and the serial commands comming from the python script will not be read. The next access to the Anaconda prompt and the Jupyter Notebook should be via the Anaconda Navigator, after running it as administrator and while the Arduino script is running.

You can deactivate your virtual environment typing in the Anaconda prompt
```
conda deactivate
```

### **Configuring the Arduino IDE**
Once you installed the Arduino IDE, you can download the zip file of the VarSpeedServo library available [here](https://github.com/netlabtoolkit/VarSpeedServo) and install it by following [this guide](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries).
Now use the file "serial-port-command-receiver.cpp" in the Arduino IDE.

### **Assembling the circuit**
The following circuit was simulated using the [Wokwi](https://wokwi.com/projects/new/arduino-uno) online platform, and considering that the four servomotors are already fitted to the robotic arm, the circuit of this image bellow must be physically assembled.

<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/wokwi.PNG?raw=true" style="width: 600px; height: 420px;"></p>

If you want to save time, use our pin configuration as shown in the circuit:

    Gripper -> Pin 2
    Lower Arm -> Pin 3
    Upper Arm -> Pin 4
    Rotating Base -> Pin 5

If you want to change the pins, just make sure the digital pins on the Arduino board receiving the signal jumpers from the servomotors are constant values specified in the file "serial-port-command-receiver.cpp" as "#define something_pin".

For example, if the servomotor that controls the robotic arm's gripper is connected to digital pin 2 on the Arduino board, the definition in the code should be "#define gripper_pin 2".

Select the right serial port within the IDE, compile and send the code to the board.

### **Using the main algorithm**
While the Arduino script is running, run Anaconda Navigator as administrator and launch the Anaconda prompt. Running as admin also applies if you are using another IDE.

Now activate your virtual environment, go back to the Navigator interface and launch Jupyter Notebook. Don't forget to activate your Kernel again.

Download the following files and put them in the same directory as the "weight-hand-segmentation-v11.pt" file.

    main.py
    robotic_arm.py
    detection_infos.py
    interface.py

Locate the COM variable in the "main.py" file and change it to the serial port currently connected to the arduino (check the Arduino IDE in case you don't remember).

For example, if your port is COM3, you need to set the COM variable to 3 and so on.

Now you can run the code and control the robotic arm by the movements of your hand.

To facilitate the process of acquiring the files, you can also download this entire project.

# Examples

The following images are examples that show what approximate positions the hand must be in for detections to be made correctly.
<h3 align="center"><b>Open hand</b></h3>
<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/open%20hand.png?raw=true" style="width: 600px; height: 420px;"></p>

<h3 align="center"><b>Closed hand</b></h3>
<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/closed%20hand.png?raw=true" style="width: 600px; height: 420px;"></p>

# Authors

- [Kennymar Bezera de Oliveira](https://github.com/KennymarOliveira)
- [Thiago Vinicius Cardoso Lopes](https://github.com/thiagoclopes)
<!---
- [Geraldo Almeida Neto](https://github.com/geraldoAn)
- [Franklin Thiago Pereira Correia](https://github.com/FranklynThP)
-->

Supervisor: [Helton Maia](https://heltonmaia.github.io/heltonmaia/) 

# Acknowledgements

Project developed for educational purposes at the Automation and Robotics Laboratory (LAR) of the School of Science and Technology (ECT) of the Federal University of Rio Grande do Norte (UFRN).
