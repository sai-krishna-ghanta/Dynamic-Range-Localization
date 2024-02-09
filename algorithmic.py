import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, id, x, y, std_noise):
        self.id = id
        self.x = x
        self.y = y
        self.neighbors = []
        self.leader_bid = None
        self.leader_id = None
        self.reference_robots = None
        self.std_noise = std_noise  

    def update_neighbors(self, robots):
        self.neighbors = [robot for robot in robots if self.distance_to(robot) <= 2 and robot.id != self.id]

    def distance_to(self, other_robot):
        true_distance_squared = (self.x - other_robot.x)**2 + (self.y - other_robot.y)**2
        # Generate noise with a lower bound to prevent negative squared distance
        lower_bound = -np.sqrt(true_distance_squared) / self.std_noise
        noise = self.std_noise * np.clip(np.random.randn(), lower_bound, None)
        noisy_distance_squared = true_distance_squared + noise
        return np.sqrt(noisy_distance_squared)

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
            robot = Robot("rb" + str(i), x, y, self.std_noise)
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

    def triangulate_positions(self, leader, ref_a, ref_b, other_robot):
        x1, y1 = self.new_coordinate_positions[leader.id]
        x2, y2 = self.new_coordinate_positions[ref_a.id]
        x3, y3 = self.new_coordinate_positions[ref_b.id]
        
        d1 = other_robot.distance_to(leader)
        d2 = other_robot.distance_to(ref_a)
        d3 = other_robot.distance_to(ref_b)
        A = 2*x2 - 2*x1
        B = 2*y2 - 2*y1
        C = d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2
        D = 2*x3 - 2*x2
        E = 2*y3 - 2*y2
        F = d2**2 - d3**2 - x2**2 + x3**2 - y2**2 + y3**2
        
        x = (C*E - F*B) / (E*A - B*D)
        y = (C*D - A*F) / (B*D - A*E)
        
        return x, y

    def calculate_new_positions(self, leader, reference_robots):
        z_la = leader.distance_to(reference_robots[0])
        z_lb = leader.distance_to(reference_robots[1])
        z_ab = reference_robots[0].distance_to(reference_robots[1])

        x_a = (z_ab**2 + z_la**2 - z_lb**2) / (2 * z_ab)
        x_b = (z_la**2 - z_ab**2 -z_lb**2) / (2 * z_ab)
        y_l = np.sqrt(z_la - x_a**2)

        self.new_coordinate_positions[leader.id] = (0, y_l)
        self.new_coordinate_positions[reference_robots[0].id] = (-abs(x_a), 0)  
        self.new_coordinate_positions[reference_robots[1].id] = (abs(x_b), 0)   

        for robot in self.robots:
            if robot not in [leader] + reference_robots:
                x, y = self.triangulate_positions(leader, reference_robots[0], reference_robots[1], robot)
                self.new_coordinate_positions[robot.id] = (x, y)


    def calculate_mse(self):
        errors = []
        for robot in self.robots:
            for neighbor in robot.neighbors:
                true_distance = np.sqrt((self.true_positions[robot.id][0] - self.true_positions[neighbor.id][0])**2 + 
                                        (self.true_positions[robot.id][1] - self.true_positions[neighbor.id][1])**2)
                new_distance = np.sqrt((self.new_coordinate_positions[robot.id][0] - self.new_coordinate_positions[neighbor.id][0])**2 +
                                    (self.new_coordinate_positions[robot.id][1] - self.new_coordinate_positions[neighbor.id][1])**2)
                error = (true_distance - new_distance)**2
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
std_noise = 0.0000001
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