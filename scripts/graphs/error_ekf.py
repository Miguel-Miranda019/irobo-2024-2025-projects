import numpy as np
import matplotlib.pyplot as plt

class EuclideanError:
    def __init__(self, npz_filepath) -> None:
        self.data = np.load(npz_filepath)

        self.times = self.data['times']
        self.errors = self.data['errors']
        
        print(len(self.times))
        print(len(self.errors))
        
    def plot_error_results(self):
        plt.plot(self.times, self.errors, label='Euclidean error throughout time')
        plt.xlabel('Time (s)')
        plt.ylabel('Euclidean error (m)')
        plt.legend()
        plt.tight_layout()
        plt.savefig('/home/miguel/catkin_ws/src/turtlebot3_datasets/graphs/error_plot2.png', dpi=300)
        plt.close()

if __name__ == '__main__':
    # Path to the .npz file containing saved data
    npz_filepath = '/home/miguel/catkin_ws/src/turtlebot3_datasets/data/ekf_gt_error_data.npz'

    # Create the error calculator with the loaded data
    position_graph = EuclideanError(npz_filepath)

    # Plot the results after processing the data
    position_graph.plot_error_results()
    

