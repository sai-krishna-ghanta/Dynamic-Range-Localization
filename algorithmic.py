import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.neighbors = []
        self.leader_bid = None
        self.leader_id = None
        self.reference_robots = None
    
    def update_neighbors(self, robots):
        self.neighbors = [robot for robot in robots if self.distance_to(robot) <= 2 and robot.id != self.id]
    
    def distance_to(self, other_robot):
        return np.sqrt((self.x - other_robot.x)*2 + (self.y - other_robot.y)*2)

class Environment:
    def __init__(self, num_robots, std_noise):
        self.num_robots = num_robots
        self.std_noise = std_noise
        self.robots = []
        self.true_positions = {}
        self.estimated_positions = {}

    def initialize_robots(self):
        for i in range(self.num_robots):
            x = np.random.uniform(0, 10)
            y = np.random.uniform(0, 10)
            robot = Robot("fb" + str(i), x, y)
            self.robots.append(robot)
            self.true_positions[robot.id] = (x, y)
    
    def leader_election(self):
        for robot in self.robots:
            robot.update_neighbors(self.robots)
            robot.leader_bid = len(robot.neighbors)
        leader = max(self.robots, key=lambda x: x.leader_bid)
        leader.leader_id = leader.id
        return leader
    
    def select_reference_robots(self, leader):
        sorted_neighbors = sorted(leader.neighbors, key=lambda x: leader.distance_to(x))
        reference_robots = sorted_neighbors[:2]
        leader.reference_robots = reference_robots
        return reference_robots
    
    def create_coordinate_system(self, leader, reference_robots):
        x_a, y_l = 0, 0
        x_b = leader.distance_to(reference_robots[0])
        x_b = np.sqrt(x_b*2 - (self.std_noise * np.random.randn())*2)  # Adding noise
        y_l = np.sqrt(leader.distance_to(reference_robots[1])*2 - x_b*2)
        return x_a, x_b, y_l

    def estimate_positions(self, leader, reference_robots):
        for robot in self.robots:
            if robot.id == leader.id:
                self.estimated_positions[robot.id] = (0, leader.y)
            elif robot.id in [r.id for r in reference_robots]:
                if robot.id == reference_robots[0].id:
                    self.estimated_positions[robot.id] = (reference_robots[1].x, 0)
                else:
                    self.estimated_positions[robot.id] = (reference_robots[0].x, 0)
            else:
                x = (self.true_positions[reference_robots[0].id][0] + self.true_positions[reference_robots[1].id][0]) / 2
                y = np.sqrt(leader.distance_to(robot)*2 - (x - leader.x)*2)
                self.estimated_positions[robot.id] = (x, y)
    
    def estimate_relative_positions(self):
        relative_positions = {}
        for robot in self.robots:
            if robot.id == leader.id:
                relative_positions[robot.id] = (0, 0)  # Leader relative position is (0, 0)
            else:
                leader_x, leader_y = self.estimated_positions[leader.id]
                robot_x, robot_y = self.estimated_positions[robot.id]
                relative_x = robot_x - leader_x
                relative_y = robot_y - leader_y
                relative_positions[robot.id] = (relative_x, relative_y)
        return relative_positions

    def calculate_mse(self):
        errors = []
        for robot in self.robots:
            true_x, true_y = self.true_positions[robot.id]
            estimated_x, estimated_y = self.estimated_positions[robot.id]
            error = ((true_x - estimated_x)*2 + (true_y - estimated_y)*2) / 2
            errors.append(error)
        mse = np.mean(errors)
        return mse

    def plot_robots(self):
        for robot in self.robots:
            plt.scatter(robot.x, robot.y, label=robot.id)
            plt.annotate(robot.id, (robot.x, robot.y))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Robot Positions')
        plt.legend()
        plt.grid(True)
        plt.show()

# Simulation parameters
num_robots = 15
std_noise = 0.1
num_iterations = 5

for i in range(num_iterations):
    # Initialize environment
    env = Environment(num_robots, std_noise)
    env.initialize_robots()

    # Leader election
    leader = env.leader_election()
    print("Iteration:", i+1)
    print("Leader:", leader.id)

    # Select reference robots
    reference_robots = env.select_reference_robots(leader)
    print("Reference Robots:", [robot.id for robot in reference_robots])

    # Create coordinate system
    x_a, x_b, y_l = env.create_coordinate_system(leader, reference_robots)
    print("Coordinate System:")
    print("x_a:", x_a)
    print("x_b:", x_b)
    print("y_l:", y_l)

    # Estimate positions
    env.estimate_positions(leader, reference_robots)

    # Estimate relative positions
    relative_positions = env.estimate_relative_positions()
    print("Estimated Relative Positions:")
    for robot_id, relative_pos in relative_positions.items():
        print(f"{robot_id}: {relative_pos}")

    # Calculate MSE
    mse = env.calculate_mse()
    print("Mean Squared Error:", mse)

    # Plot robots
    env.plot_robots()