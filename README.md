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

 1. **Setup of the Environment:** A predetermined number of robots are arranged at random in a predetermined area to form the environment. The uncertainty in position measurements is also reflected in the configuration of the noise standard deviation

## Using Robotarium
This approach extends the Robotarium framework, adding initialization and leader election function to the library. This approach is an improvement of algorithmic approach with Robotarium framework. 

## The Installation
The implementation of the proposed work is easy to perform. 

1. **Clone the Repository**:
```git clone https://github.com/sai-krishna-ghanta/Dynamic-Range-Localization.git```

2. **Install Dependencies**:
```pip install numpy matplotlib```

3. **Run the Scripts**:
```python a_robotarium.py ```
```python algorithmic.py```


