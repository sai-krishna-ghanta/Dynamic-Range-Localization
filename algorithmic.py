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
        return np.sqrt((self.x - other_robot.x)**2 + (self.y - other_robot.y)**2)

class Environment:
    def __init__(self, num_robots, std_noise):
        self.num_robots = num_robots
        self.std_noise = std_noise
        self.robots = []
        self.true_positions = {}
        self.new_coordinate_positions = {}

    def initialize_robots(self):
        self.robots = []
        self.true_positions = {}
        self.new_coordinate_positions = {}
        for i in range(self.num_robots):
            x = np.random.uniform(0, 10)
            y = np.random.uniform(0, 10)
            robot = Robot("rb" + str(i), x, y)
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

    def calculate_new_positions(self, leader, reference_robots):
        z_la = leader.distance_to(reference_robots[0])
        z_lb = leader.distance_to(reference_robots[1])
        z_ab = reference_robots[0].distance_to(reference_robots[1])
        x_a, x_b, y_l = self.calculate_positions(z_la, z_lb, z_ab)
        self.new_coordinate_positions[leader.id] = (0, 0)  # Leader at origin
        self.new_coordinate_positions[reference_robots[0].id] = (x_a, 0)
        self.new_coordinate_positions[reference_robots[1].id] = (x_b, np.sqrt(abs(z_lb**2 - x_b**2)))

        for robot in self.robots:
            if robot not in [leader] + reference_robots:
                self.new_coordinate_positions[robot.id] = (np.random.uniform(-5, 5), np.random.uniform(-5, 5))  # Placeholder

    def calculate_positions(self, z_la, z_lb, z_ab):
        x_a = (z_ab**2 + z_la**2 - z_lb**2) / (2 * z_ab)
        x_b = (z_ab**2 - z_la**2 + z_lb**2) / (2 * z_ab)
        y_l = np.sqrt(abs(z_la**2 - x_a**2))
        return x_a, x_b, y_l
    
    def calculate_new_positions_with_noise(self):
        # Simulate measurement noise and recalculate positions
        for robot in self.robots:
            dx = self.std_noise * np.random.randn()
            dy = self.std_noise * np.random.randn()
            self.new_coordinate_positions[robot.id] = (robot.x + dx, robot.y + dy)


    def calculate_mse(self):
        errors = []
        for robot_id, new_pos in self.new_coordinate_positions.items():
            true_pos = self.true_positions[robot_id]
            error = np.sum((np.array(true_pos) - np.array(new_pos))**2)
            errors.append(error)
        mse = np.mean(errors)
        return mse

    def plot_robots(self, iteration, leader_id, reference_robot_ids):
        fig, ax = plt.subplots(1, 2, figsize=(12, 6), sharey=True)
    
        ax[0].set_title("Ground Truth Positions")
        for robot_id, (x, y) in self.true_positions.items():
            ax[0].scatter(x, y, label=f"{robot_id}", alpha=0.6)
            ax[0].text(x, y, robot_id, fontsize=8)
        ax[0].grid(True)

        ax[1].set_title("New Coordinate System Positions")
        for robot_id, (x, y) in self.new_coordinate_positions.items():
            if robot_id == leader_id:
                marker = 'o'
                color = 'red'
                label = 'Leader'
            elif robot_id in reference_robot_ids:
                marker = '^'
                color = 'green'
                label = 'Reference'
            else:
                marker = 's'
                color = 'blue'
                label = 'Other'
            ax[1].scatter(x, y, marker=marker, color=color, label=label if robot_id == leader_id or robot_id in reference_robot_ids else None)
            ax[1].text(x, y, robot_id, fontsize=8)
        ax[1].grid(True)
    
        from matplotlib.lines import Line2D
        custom_lines = [Line2D([0], [0], marker='o', color='red', lw=0, label='Leader'),
                    Line2D([0], [0], marker='^', color='green', lw=0, label='Reference'),
                    Line2D([0], [0], marker='s', color='blue', lw=0, label='Other')]
        ax[1].legend(handles=custom_lines, loc='best')
    
        plt.suptitle(f'Iteration {iteration}')
        plt.tight_layout()
        plt.show()


num_robots = 15
std_noise = 0.1
num_iterations = 5

for i in range(num_iterations):
    env = Environment(num_robots, std_noise)
    env.initialize_robots()

    leader = env.leader_election()
    reference_robots = env.select_reference_robots(leader)
    reference_robot_ids = [robot.id for robot in reference_robots]

    env.calculate_new_positions(leader, reference_robots)
    mse = env.calculate_mse()

    print(f"Iteration {i+1}:")
    print(f"Mean Squared Error (MSE): {mse:.3f}")
    print("Ground Truth Positions:", env.true_positions)
    print("New Coordinate System Positions:", env.new_coordinate_positions)
    env.plot_robots(i + 1, leader.id, reference_robot_ids)