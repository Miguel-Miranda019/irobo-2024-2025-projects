import pandas as pd
import matplotlib.pyplot as plt

class GTPositionPlot:
    def __init__(self, csv_filepath) -> None:
        # Load data from the CSV file
        self.data = pd.read_csv(csv_filepath)

        # Extract relevant columns for Ground Truth
        self.times = self.data['__time'].values
        self.gt_position_x = self.data['/gtInfo/pose/pose/position/x'].values
        self.gt_position_y = self.data['/gtInfo/pose/pose/position/y'].values

        # Extract relevant columns for AMCL
        self.amcl_position_x = self.data['/robot_pose_with_covariance_amcl/pose/pose/position/x'].values
        self.amcl_position_y = self.data['/robot_pose_with_covariance_amcl/pose/pose/position/y'].values

        # Print the length of each data array to verify
        print(f"Number of data points: {len(self.times)}")

    def plot_gt_position(self):
        plt.figure()

        # Plotting the Ground Truth X and Y positions over time
        plt.subplot(2, 1, 1)
        plt.plot(self.times, self.gt_position_x, label='Ground Truth Position - Axis X', color='r', linewidth=1.0)  # Thinner line
        plt.plot(self.times, self.amcl_position_x, label='AMCL Position - Axis X', color='b', linestyle='--', linewidth=1.0)  # Thinner dashed line

        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.title('Ground Truth vs AMCL X Position Over Time')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.times, self.gt_position_y, label='Ground Truth Position - Axis Y', color='r', linewidth=1.0)  # Thinner line
        plt.plot(self.times, self.amcl_position_y, label='AMCL Position - Axis Y', color='b', linestyle='--', linewidth=1.0)  # Thinner dashed line

        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.title('Ground Truth vs AMCL Y Position Over Time')
        plt.legend()

        # Adjust layout and save the plot
        plt.tight_layout()
        plt.savefig('/home/raquel/catkin_ws/src/turtlebot3_datasets/graphs/amcl/gt_amcl_position_plot.png', dpi=300)
        plt.close()

if __name__ == '__main__':
    # Path to the CSV file containing the GT data
    csv_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/gt.csv'

    # Create the Ground Truth position plot object with the loaded data
    gt_position_plot = GTPositionPlot(csv_filepath)

    # Plot the GT position results
    gt_position_plot.plot_gt_position()
