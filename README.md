# UR5 with Epick Gripper: Object Pick and Place Simulation

This project simulates a UR5 robotic arm with a Epick suction gripper, performing autonomous object grasping and placement tasks in the PyBullet environment. Using inverse kinematics (IK) for precise arm control and synchronized joint control for realistic gripper motion, the robot grasps cubes from random positions and places them on a tray.

---

## Results Showcase  

### Grasping and Placing Demo  
Watch the UR5 robot in action as it grasps and places objects onto a tray:  



---

## Usage Instructions  

### 1. Setup the Environment  
- Install the required dependencies:  
    ```bash  
    pip install pybullet  

### 2. Run the Simulation
- This will initialize the simulation, and the robot will begin performing the object grasping and placement task.
- Launch the program using the following command:
    ```bash
    python ur5_epick.py

### 3. PyBullet GUI
- Once the simulation is running, you can use the PyBullet GUI to observe the robot’s actions in real-time. The interface allows you to track the movement of the robotic arm, gripper, and cubes as they interact within the environment.

- TO ensure the simulation can be run, should be use gpu if there is no mesa in your PC

```
export LIBGL_ALWAYS_INDIRECT=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

---

## Key Features

- **UR5 Robotic Arm Simulation**: The UR5 robotic arm uses inverse kinematics (IK) to move to the correct positions and accurately control its joints for task execution.
- **Epick suction Gripper**: The gripper is simulated using create/detach joint constrain to monitor the suction on/off command
- **Dynamic Object Placement**: Cubes are randomly generated in the environment, making the task more dynamic and representative of real-world applications.
- **Real-Time Interaction**: The PyBullet GUI allows for interactive and visual monitoring of the robot’s actions, providing a comprehensive view of the simulation.

---

## Resources & References

- **PyBullet**: PyBullet is a physics engine for simulating robots and their environments. You can find more details and documentation on the official website of [PyBullet](https://pybullet.org/).


