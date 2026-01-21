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

## 3. Requirements

Please complete [turtle_eight.py](turtle_eight.py) and make the turtle create figure-8 patterns as shown in the example GIF.

### 3.1. General rules

- The turtle will start at the defult pose (center of the canvas facing towards right).
- The turtle is expected to draw the top circle **couterclockwisely** with the radius of **1**.
- The turtle is expected to draw the bottom circle **clockwisely** with the radius of **2**.
- Change turtle's direction **every time** when it gets back to the starting location.
- The angular speed of the turtle has to be fixed at **$\frac{\pi}{4}$ rad/s** (changing direction should not affect the scale of the speed).
- The turtle's trajectory (figure-8 pattern) needs to be inscribed within the two red squares (See image below for an example).

 ![fig8_example](/images/fig8_example.gif)

> [!WARNING]
> Please focus on the code wrapped around the comments showing below.
> Change the code out of the scope at your own risk.

```python
### START CODING HERE ###

### END CODING HERE ###
```

### 3.2. Create a node

- Start your node with name: `/<your_name>/turtle8`

> [!WARNING]
> Your assignment will not be graded if `<your_name>` is unidentifiable.

### 3.3. Keep an eye on the motion status

- Set up a subscriber and "listen" to the appropriate topic which contains the turtle's pose and velocity.
  Take out the message embedded in the topic.
- Print the turtle's pose to the screen at a frequency of 5Hz.
  Please encode the turtle's pose with the format below:

  ```python
  f"Turtle's pose: \nposition x: {position_x}, position y: {position_y}, orientation z: {orientation_z}"
  ```

### 3.4. Plan turtle's trajectory
>
> [!IMPORTANT]
> Given the angular velocity, $\omega$ (rad/s) and the radius, $R$ (m), of a **circular motion**, what would be a good linear velocity $v$ (m/s) for the turtle?

- Please write down the math equation of the linear velocity in the [README](README.md).
- Please plug in the required radius of the circles to the equation.
  Calculate and write down the linear velocity values for these circles in the [README](README.md).
- Please calculate the time needed for the turtle to finish one lap of a circle.
  Analyze the temporal difference between travelling the top circle and the bottom circle.
  And write your analysis down in [README](README.md).

### 3.5. Draw figure-8

- Set up a publisher (and optionally, a timer) to "talk" about the turtle's velocity commands under an appropriate topic.
- Embed the calculated velocity to the message with the right type for the topic.
- Switch the direction of the turtle at the right instant.
- The turtle has to draw identical 8️⃣ figures from lap to lap.

![example_fig8](turtlesim_play_pkg/images/example_fig8.gif)

### 3.6. Upload your turtle's footprints

- Let your turtle complete at five figures then display your turtle's figure-8 footprints in [README](README.md).

### 3.7 AI Policies

Please acknowledge AI's contributions according to the policies in the syllabus.
