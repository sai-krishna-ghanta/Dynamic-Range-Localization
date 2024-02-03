# Dynamic Range Localization

For Static Visualizations - run jupyter notebooks


For Interactive Visualizations - run python files or upload code to robotarium


## Objective
The objective of this work is to enable robots to autonomously establish a common coordinate system and accurately estimate their positions relative to each other using only range information. This is achieved through two different approaches:

### 1. **Algorithmic Approach**
### 2. **Robotarium Approach**

## Algorithmic Approach

### Overview
The code models an environment in which robots operate with the goal of estimating their positions in relation to a leader robot. The ultimate objective is to evaluate the location estimate algorithm's accuracy and comprehend the behavior of the system.

 1. **Setup of the Environment:** A predetermined number of robots are arranged at random in a predetermined area to form the environment. The uncertainty in position measurements is also reflected in the configuration of the noise standard deviation.
 2. **Leader Election:** In the leader election phase, each robot evaluates its local neighborhood to determine the number of neighboring robots it can communicate with. The robot with the highest number of neighbors is identified as the leader due to its significant connectivity within the network. Being the leader implies a central role in the coordination and decision-making processes within the group of robots.
 3. **Reference Robot Selection:** Following election, the leader chooses reference robots from among its close neighbors. Within the robot network, these reference robots act as fixed points to create a local coordinate system.  Reference robots are usually selected according to how close they are to the leader and how solid their positions are in relation to it. By choosing reference robots strategically, the leader ensures the stability and accuracy of the coordinate system used for position estimation.
 4. **Coordinate System Creation:** With the reference robots identified, the leader proceeds to establish a local coordinate system. This system provides a frame of reference for position estimation, enabling robots to determine their locations relative to the leader and reference points. The distances between the leader and reference robots define the axes of this coordinate system. Typically, the x-axis is aligned with the line connecting the leader and one reference robot, while the y-axis is perpendicular to the x-axis, maintaining a right-handed orientation.
 5. **Position Estimation:** Trilateration techniques involve utilizing the known distances between a robot and multiple reference points (in this case, the leader and reference robots) to estimate the robot's position (find each robot's x and y coordinates). By applying trigonometric calculations within the established coordinate system, accurate localization of each robot within the network is achieved..
 6. **Relative Position Calculation:** The relative positions of each robot with respect to the leader are computed when the positions of all robots are estimated. In order to determine each robot's position in relation to the leader, the leader's coordinates must be subtracted from each robot's estimated coordinates.
 7. **MSE Calculation:** The Mean Squared Error (MSE) measure is used in error analysis to evaluate the location estimate process' accuracy. The Mean Squared Error (MSE) measures the average squared difference between two sets of positions by comparing the actual positions of robots with their estimated positions.
 8. **Visualization:** Seeing every robot's location on a two-dimensional layout is the last stage. This graphical representation aids in understanding the spatial arrangement of robots and validates the effectiveness of the position estimation process.

**Results:** The output for each iteration includes the leader and reference robots, along with the estimated relative positions and mean squared error (MSE). Additionally, it provides the coordinates x_a (representing one reference robot's x-coordinate), x_b (representing the other reference robot's x-coordinate), and y_l (representing the leader robot's y-coordinate in its own coordinate system). 

![95FC043B-3956-4E0F-BE6E-BD5608A9CA90](https://github.com/sai-krishna-ghanta/Dynamic-Range-Localization/assets/84200397/2d2b22b3-25ac-4b86-b9ae-e13afa94ac90)

## Using Robotarium
This approach extends the Robotarium framework, adding initialization and leader election function to the library. This approach is an improvement of algorithmic approach with Robotarium framework. 

## The Installation
The implementation of the proposed work is easy to perform. 

1. **Clone the Repository**:
```git clone https://github.com/sai-krishna-ghanta/Dynamic-Range-Localization.git```

2. **Install Dependencies**:
```pip install numpy scipy matplotlib```

3. **Run the Scripts**:
```python a_robotarium.py ```
```python algorithmic.py```


