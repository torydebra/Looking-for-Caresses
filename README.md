# Looking-for-Caresses
Project for Social Robotics 2018/19 course of Robotics Engineering.
## Description
MiRo is a small pet-like robot intended to be a companion. In this project, MiRo behave as companion, standing on the desk while the user is working (for example, at pc). 
* **Task A: Awakening** Randomly, MiRo awakes to look for caresses from the user. The more the time pass, the more MiRo feels lonely so the probability that it wakes up increase. Also, if the user touches MiRo, it awakes immediately.
* **Task Aa: Face detection**
Awakened MiRo looks for the user face (Aa face detection task), turning on itself until the face is detected with both cameras (so, the face is in front of the robot).
* **Task B: Approaching** 
Then it comes near to the user. The user must put a hand close to MiRo, to make him detect it with the sonar. If the hand is near enough to the muzzle (where there is the sonar) MiRo stop and interaction begins. 
* **Task C: Interaction**
The user can choose to caresses MiRo on the back to make him happy. After some time, Miro will be satisfied and he will return to sleep. If the user don't want to interact, he must pat MiRo on head, and MiRo will go to sleep immediately.

More detail  [here](https://github.com/EmaroLab/Looking-for-Caresses/blob/master/tex/SocialRepo.pdf)

## How To Run
### Prerequisites
* [ROS](http://www.ros.org/)  (code tested only on Kinetic version on Ubuntu 16 machine)
##### For face_detection
* [Opencv 3.4](https://docs.opencv.org/3.3.0/d7/d9f/tutorial_linux_install.html)
* Run in terminal:
    ```bash
    sudo apt-install ros-kinetic-opencv3 #(should be already installed with previous point)
    sudo apt-install ros-kinetic-opencv-apps
    ```
##### For touch pattern recognition
* Run in terminal:
    ```bash
    sudo pip install --ignore-installed tensorflow
    sudo apt install python-sklearn
    sudo pip install keras
    ```
### To Run 
Be sure to have correctly setup your machine and Miro, as described here [MIRO_setup](https://github.com/EmaroLab/MIRO/blob/master/miro_setup_quick/MIRO_setup)
* Download folder and compile with ```catkin_make ```
* Run supports nodes and main singularly, in different terminals:
   * Face Detection  
    This will run two window for the two cameras
        ```bash
         roslaunch look_caresses_pkg face_detect_double.launch
        ```
    * Datainput.py for pattern recognition
        ```bash
        cd [YOUR_PATH]/Looking-for-Caresses/src/look_caresses_pkg/src
        ./DataInput.py robot=rob01
        ```
    * Coordinator (actual main)
        ```bash
        rosrun look_caresses_pkg Coordinator
        ```

## Credits
#### Developers
* [tori](https://github.com/torydebra)
* [fafux](https://github.com/fafux)
* The Pattern recognition node (DataInput.py) is taken from [here](https://github.com/EmaroLab/Miro_SocialRobot/blob/master/README.md)
* [MIRO website](http://labs.consequentialrobotics.com/)
