# Hybrid Control for Robotic Arms: Combining Gesture Recognition and Voice Commands

This project encompasses computer vision, machine learning, 3D printing, and Arduino integration, with the primary goal of enabling robotic arms to replicate the movements of a human arm. The project consists of a PyQt6 application where the user can choose to control the robotic arm either through gestures or voice commands. If the user prefers gesture control, the application will use the OpenCV computer vision library to capture real-time images using a webcam or smartphone, and relevant information from the user's hand will then be detected by one of our neural networks trained based on the YOLOv8 architecture. However, if the user prefers voice command control, the application will use the Librosa library for recording and processing, and then our custom audio classification neural network will be used to recognize the command. Regardless of the control method, the movement of the robotic arm, which was 3D-printed, is executed by the Arduino according to serial port commands sent by a Python script using the Pyserial module.

[Project video](https://github.com/heltonmaia/ECT-proj-roboticArm/assets/4681481/2796c126-4182-4c66-be8b-e16110343908)

# How to use
### **Requirements**
  - Python 3.10 ([Install Guide](https://www.python.org/downloads/release/python-3100/))
  - Visual Studio Code ([Setup Guide](https://code.visualstudio.com/docs/setup/setup-overview)) or any IDE of your choice
  - A Python virtual environment ([How to do it on VS Code](https://code.visualstudio.com/docs/python/environments))
  - Arduino IDE ([Install Guide](https://www.arduino.cc/en/software))
  - Webcam or a smartphone
    
### **Configuring the project**
Download the "Hybrid-Robotic-Arm-Control.zip" file and extract it to a folder with the same name. This will be the main project directory.

Open the folder in your preferred IDE.

The weight files for both neural networks to function should be in the "src/misc/" directory inside the main folder.

There will be a "readme.txt" file with download links.

If you have already created a virtual environment, activate it.

### **Installing Necessary Modules**

After creating the virtual environment, it's time to install all the necessary modules.

Download the "requirements.txt" file available in this repository. Open the terminal in the IDE and type this command, replacing it with the correct directory of the file:

```python
pip install -r C:\Path\To\The\File\requirements.txt
```

### **Accessing the Camera and Microphone**

If you don't have dedicated camera or microphone, you can use the Droidcam mobile app to use your smartphone as both devices.
  - Install the Droidcam app (Android or IOS): [Download Page](https://www.dev47apps.com)
  - Install the Droidcam client on Windows: [Download Page](https://www.dev47apps.com/droidcam/windows/)
  - Then follow the instructions to connect: [Connection Guide](https://www.dev47apps.com/droidcam/connect/)

Configuring the Arduino IDE

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

### **Using the App**

Run the "roboticarm_qt.py" file. If you encounter any errors related to DLLs, copy the "libomp140.x86_64.dll" file available [here](https://github.com/heltonmaia/ECT-proj-roboticArm/tree/main/fixes) to your system's "C:\Windows\System32\" folder.

After running the application, go to the Setup screen and enter the serial port corresponding to your Arduino.

Choose the capture device (usually 0 or 1, but it may differ if you have multiple cameras and microphones connected to your computer).

Select the desired control method, and now everything is ready.

Start the robotic arm control through the main menu.

# Examples

### **Gestures**

The following images are examples that show what approximate positions the hand must be in for detections to be made correctly.

<h3 align="center"><b>Open hand</b></h3>
<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/open%20hand.png?raw=true" style="width: 600px; height: 420px;"></p>

<h3 align="center"><b>Closed hand</b></h3>
<p align="center"><img src="https://github.com/heltonmaia/ECT-proj-roboticArm/blob/main/images/closed%20hand.png?raw=true" style="width: 600px; height: 420px;"></p>

### **Voice commands**

These are the voice commands that our audio classification neural network currently identifies.

```
up, down, left, right, open, close
```

# Authors
- [Kennymar Oliveira](https://github.com/KennymarOliveira)
- [Thiago Lopes](https://github.com/thiagoclopes)
- [Virna Aguiar](https://github.com/virnaaguiaar)

Supervisor: [Helton Maia](https://heltonmaia.github.io/heltonmaia/) 

# Acknowledgements

Project developed for educational purposes at the Automation and Robotics Laboratory (LAR) of the School of Science and Technology (ECT) of the Federal University of Rio Grande do Norte (UFRN).
