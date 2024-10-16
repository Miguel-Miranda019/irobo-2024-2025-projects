import numpy as np
import matplotlib.pyplot as plt

class PositionWithCovGraph:
    def __init__(self, npz_filepath) -> None:
        self.data = np.load(npz_filepath)

        self.gt_pose_x = self.data['gt_pose_x']
        self.gt_pose_y = self.data['gt_pose_y']
        self.amcl_pose_x = self.data['amcl_pose_x']
        self.amcl_pose_y = self.data['amcl_pose_y']
        self.times = self.data['times']
        #self.times = self.times[:-1]
        self.uncertainties_x = self.data['uncertainties_x']
        #self.uncertainties_x = self.uncertainties_x[:-1]
        self.uncertainties_y = self.data['uncertainties_y']
        #self.uncertainties_y = self.uncertainties_y[:-1]
        self.gt_poses = []
        self.amcl_poses = []
        self.errors = []
        self.gt_time = np.linspace(0, 49, 50)
        self.gt_time = self.gt_time.astype(int)

        self.calculate_error()
        
    def calculate_error(self):
        n_gt_poses = len(self.gt_pose_x)
        n_amcl_poses = len(self.amcl_pose_x)
        
        print(n_amcl_poses, n_gt_poses)
        
        for i in range(len(self.amcl_pose_x)):
            amcl_pose = [self.amcl_pose_x[i], self.amcl_pose_y[i]]
            self.amcl_poses.append(amcl_pose)
        
        difference = (n_gt_poses / n_amcl_poses)
        index = 0
        for i in range(n_amcl_poses):
            # Ensure we don't go out of bounds
            gt_index = int(round(index))  # Use rounding to get the nearest index
            gt_pose = [self.gt_pose_x[gt_index], self.gt_pose_y[gt_index]]
            self.gt_poses.append(gt_pose)
            index += difference  # Move to the next index based on the calculated difference
        
        print("hi", len(self.amcl_poses))
        print(len(self.gt_poses))
        
        self.cumulative_errors = [0] * len(self.amcl_poses)
        for i in range(len(self.amcl_poses)):
            error = np.sqrt((self.gt_poses[i][0] - self.amcl_poses[i][0])**2 + (self.gt_poses[i][1] - self.amcl_poses[i][1])**2)
            self.errors.append(error)
            self.cumulative_errors[i] = self.cumulative_errors[i-1] + error
        
        print(len(self.cumulative_errors))
        
    def plot_position_results(self):
        plt.figure()
        
        plt.subplot(2, 1, 1)
        plt.plot(self.times, self.amcl_pose_x, label='Estimated AMCL Position - Axis X')
        print(len(self.uncertainties_x))
        
        plt.fill_between(self.times,
                        np.array(self.amcl_pose_x) - 2 * np.array(self.uncertainties_x),
                        np.array(self.amcl_pose_x) + 2 * np.array(self.uncertainties_x),
                        color='b', alpha=0.2, label='95% Confidence Interval (X)')
        
        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.times, self.amcl_pose_y, label='Estimated AMCL Position - Axis Y')
        
        # Confidence interval: [amcl_position - 2*uncertainty, amcl_position + 2*uncertainty]
        plt.fill_between(self.times,
                        np.array(self.amcl_pose_y) - 2 * np.array(self.uncertainties_y),
                        np.array(self.amcl_pose_y) + 2 * np.array(self.uncertainties_y),
                        color='r', alpha=0.2, label='95% Confidence Interval (Y)')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.legend()

        plt.tight_layout()
        plt.savefig('/home/miguel/catkin_ws/src/turtlebot3_datasets/graphs/amcl_position_plot.png', dpi=300)
        plt.close()
        
    def plot_error_results(self):
        plt.plot(self.gt_time, self.errors, label='Euclidean error throughout time')
        plt.xlabel('Time (s)')
        plt.ylabel('Euclidean error (m)')
        plt.legend()
        plt.tight_layout()
        plt.savefig('/home/miguel/catkin_ws/src/turtlebot3_datasets/graphs/amcl_error_plot.png', dpi=300)
        plt.close()
        
    def plot_cumulative_error_results(self):
        plt.plot(self.gt_time, self.cumulative_errors, label='Cumulative euclidean error throughout time')
        plt.xlabel('Time (s)')
        plt.ylabel('Cumulative euclidean error (m)')
        plt.legend()
        plt.tight_layout()
        plt.savefig('/home/miguel/catkin_ws/src/turtlebot3_datasets/graphs/amcl_cumulative_error_plot.png', dpi=300)
        plt.close()
        
if __name__ == '__main__':
    # Path to the .npz file containing saved data
    npz_filepath = '/home/miguel/catkin_ws/src/turtlebot3_datasets/data/amcl_bag_data.npz'

    # Create the error calculator with the loaded data
    position_graph = PositionWithCovGraph(npz_filepath)

    # Plot the results after processing the data
    position_graph.plot_position_results()
    position_graph.plot_error_results()
    position_graph.plot_cumulative_error_results()
    

