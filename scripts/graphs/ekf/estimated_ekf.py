import numpy as np
import matplotlib.pyplot as plt

class PositionWithCovGraph:
    def __init__(self, npz_filepath) -> None:
        self.data = np.load(npz_filepath)

        self.ekf_pose_x = self.data['ekf_pose_x']
        self.ekf_pose_y = self.data['ekf_pose_y']
        self.times = self.data['times']
        self.uncertainties_x = self.data['uncertainties_x']
        self.uncertainties_y = self.data['uncertainties_y']
        
        print(len(self.ekf_pose_x))
        print(len(self.ekf_pose_y))
        print(len(self.times))
        print(len(self.uncertainties_x))
        print(len(self.uncertainties_y))
        
    def plot_position_results(self):
        plt.figure()
        
        plt.subplot(2, 1, 1)
        plt.plot(self.times, self.ekf_pose_x, label='Estimated EKF Position - Axis X')
        
        plt.fill_between(self.times,
                        np.array(self.ekf_pose_x) - 2 * np.array(self.uncertainties_x),
                        np.array(self.ekf_pose_x) + 2 * np.array(self.uncertainties_x),
                        color='b', alpha=0.2, label='95% Confidence Interval (X)')
        
        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.times, self.ekf_pose_y, label='Estimated EKF Position - Axis Y')
        
        # Confidence interval: [ekf_position - 2*uncertainty, ekf_position + 2*uncertainty]
        plt.fill_between(self.times,
                        np.array(self.ekf_pose_y) - 2 * np.array(self.uncertainties_y),
                        np.array(self.ekf_pose_y) + 2 * np.array(self.uncertainties_y),
                        color='r', alpha=0.2, label='95% Confidence Interval (Y)')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.legend()

        plt.tight_layout()
        plt.savefig('/home/raquel/catkin_ws/src/turtlebot3_datasets/graphs/ekf/ekf_position_plot.png', dpi=300)
        plt.close()

if __name__ == '__main__':
    # Path to the .npz file containing saved data
    npz_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/ekf/ekf_bag_data.npz'

    # Create the error calculator with the loaded data
    position_graph = PositionWithCovGraph(npz_filepath)

    # Plot the results after processing the data
    position_graph.plot_position_results()
    

