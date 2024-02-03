# Dynamic Range Localization


## Objective
The objective of this work is to enable robots to autonomously establish a common coordinate system and accurately estimate their positions relative to each other using only range information. This is achieved through two different approaches:

### 1. **Algorithmic Approach**
### 2. **Robotarium Approach**

## Algorithmic Approach

### Overview
The code models an environment in which robots operate with the goal of estimating their positions in relation to a leader robot. The ultimate objective is to evaluate the location estimate algorithm's accuracy and comprehend the behavior of the system.

 1. Setup of the Environment
 2. Leader Election
 3. Reference Robot Selection
 4. Coordinate System Creation
 5. Position Estimation
 6. Relative Position Calculation
 7. MSE Calculation

## Using Robotarium
This approach extends the Robotarium framework, adding initialization and leader election function to the library. This approach is an improvement of algorithmic approach with Robotarium framework. 

## The Installation
The implementation of the proposed work is easy to perform. 

1. **Clone the Repository**:
```git clone https://github.com/sai-krishna-ghanta/Dynamic-Range-Localization.git```

2. **Install Dependencies**:
``` pip install numpy matplotlib```

3. ** Run the Scripts **:
```    python a_robotarium.py ```
``` python algorithmic.py```


