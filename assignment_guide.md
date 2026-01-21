# Figure 8 Turtlesim

## 1. Overview
Just as an orchestra creates a symphony through the coordination of many musicians, a robot is orchestrated by the Robot Operating System (**ROS**).
ROS relies on **nodes**(musicians) to perform specific tasks, such as reading sensors or controlling actuators.
When nodes need to communicate and cooperate with each other, they use **topics** (instruments). 
A topic bears **messages** (notes) allowing the nodes to respond to it.
Just like a violin section only plays violin notes, each ROS Topic is strictly constrained to a specific message type.

In this assignment, you will practice orchestrating a simulated robot to "dance" in a figure-8 pattern.
To achieve this, you will:
- Launch the Simulation: Start the `turtlesim` node to bring a robotic turtle to life.
- Create a node to interact with the `turtlesim` node.
- Subscribe to a topic to monitor the robot's motion status.
- Publish a topic with strictly formatted message to drive the robot. 


## 2. Get Started

### 2.1. Usage
1. Launch turtlesim (in a terminal)
```console
ros2 run turtlesim turtlesim_node
```
2. Test your code (in a new terminal)
```console
python3 <path_to>/turtle_eight.py
```

> [!TIP]
> You can click the play button (:arrow_forward:) in your IDE (e.g. vscode, thonny) to execute your python script.

### 2.2. Hardware
A computer with access to ROS Jazzy

### 2.3. Software
- (Recommended) Ubuntu 24.04
- (Recommended) ROS [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [turtlesim](https://index.ros.org/p/turtlesim/)
- (Optional) [VS Code](https://code.visualstudio.com/)

### 2.4. Plan Turtle's Trajectory
Please think about the following question.

> [!IMPORTANT]
> Given the angular velocity, $\omega$ and the radius ($R$) of a **circular motion**, what would be a good linear velocity $v$?

## 3. Requirements:
Please complete [turtle_eight.py](turtle_eight.py) and fulfill the following requirements.
Replace the `None`s with apt operations.

> [!WARNING]
> Please focus on the code wrapped around the comments showing below.
> Change the code out of the scope at your own risk.
```python
### START CODING HERE ###

### END CODING HERE ###
```

### 3.1. Create a node
- with name: `/<your_name>/turtle_8`

### 3.2 Keep an eye on the motion status
- Set up a subscriber and "listen" to the appropriate topic which contains the turtle's pose and velocity.
  Take out the message embedded in the topic.
- Print the turtle's pose to the screen at a frequency of 5Hz.
  Please encode the turtle's pose with the format below:
  ```python
  f"Turtle's pose: \nposition x: {position_x}, position y: {position_y}, orientation z: {orientation_z}"
  ```
1. (5%) Download and build the ROS package. 
   1. [Create a ROS workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#create-a-new-directory).
   2. Clone this repository down to the `/src` dirctory in your ROS workspace.
   3. Build `turtlesim_play_pkg` package.
      **NOTE**: you need to specify `<ros workspace path>` according to the 1st step.
      Verify if your package was downloaded to the right location and was successfully built.
      1. Open a terminal window and run following commands:
      ```console
      cd <ros workspace path>
      colcon build
      source install/local_setup.bash  # CRITICAL, or ROS can't find your package
      ```   
      2. Start `turtlesim_node` in a terminal
      ```console
      source <ros workspace path>/install/local_setup.bash
      ros2 run turtlesim turtlesim_node
      ```
      3. Open another terminal, start the `figure8_node`
      ```console
      source <ros workspace path>/install/local_setup.bash
      ros2 run turtlesim_play_pkg figure8_node
      ```
   In case of mistakes, you'll want to start over. Remove the entire ROS workspace using command: `rm -rf <ros workspace path>`
2. (80%) Complete the [figure8_node.py](turtlesim_play_pkg/turtlesim_play_pkg/figure8_node.py).
   Fill approriate operations between the commented lines:
   ```python
   ### START CODING HERE ###

   ### END CODING HERE ###
   ```
   - (20%) The turtle is expected to leave **identical** figure 8 trjectories from lap to lap.
   - (20%) The turtle's trajectory (figure 8) needs to be **inscribed** within the two squares (See image below for an example).
   - (20%) The turtle is supposed to draw the top circle couterclockwisely with a **radius of 1**.
   - (20%) The turtle is supposed to draw the bottom circle clockwisely with a **radius of 2**.
   - The **angular speed** of the turtle has to be fixed at **$$\frac{\pi}{4}$$ rad/s**.
   
   You need to determine the **linear velocity** and `/turtle1/cmd_vel` topic **publish rate** to regulate the turtle's motion.
   
   ![example_fig8](turtlesim_play_pkg/images/example_fig8.gif)

3. (10%) Let the turtle complete at least five laps then upload your figure 8 to the [images/](turtlesim_play_pkg/images/) directory.
   Illustrate Your turtle's execution below (edit next line in this [README](README.md)):
   
   ![fig8_practice](turtlesim_play_pkg/images/fig8_practice.png)
   
5. (5%) Fill the `<description>`, `<maintainer>`, `<maintainer_email>` fields with your own information in [package.xml](turtlesim_play_pkg/package.xml) and [setup.py](turtlesim_play_pkg/setup.py).
Look for the fields marked with `TODO` in these files.

## Study Resources

### Circular Motion Kinematics
Given an object is doing the circular motion in constant linear/angular velocity. 
The relationship between the linear and angular velocity is shown as the following figure, where $$r$$ is the radius of the circle.

![lin_ang_vel](https://yairshinar.com/wp-content/uploads/2018/12/c99655fa7435cc516bb40ac7daaa51c9.jpg)

### Linux Command Line Tutorial
[https://ubuntu.com/tutorials/command-line-for-beginners#1-overview](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview)

## AI Policies
Please acknowledge AI's contributions according to the policies in the [syllabus](https://linzhanguca.github.io/_docs/robotics2-2025/syllabus.pdf).
