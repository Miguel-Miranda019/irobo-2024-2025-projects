import pandas as pd
import matplotlib.pyplot as plt

class EKFErrorPlot:
    def __init__(self, csv_filepath) -> None:
        # Load data from the CSV file
        self.data = pd.read_csv(csv_filepath)

        # Extract relevant columns
        self.times = self.data['__time'].values
        self.ekf_error = self.data['/ekf_error/data'].values

        # Calculate the average error
        self.average_error = self.ekf_error.mean()

        # Print the length of each data array to verify
        print(f"Number of data points: {len(self.times)}")
        print(f"Average EKF Euclidean Error: {self.average_error:.3f} m")

    def plot_ekf_error(self):
        plt.figure()

        # Plotting the EKF Euclidean error over time
        plt.plot(self.times, self.ekf_error, label='EKF Euclidean Error', color='b')

        # Plotting the average error as a horizontal line
        plt.axhline(y=self.average_error, color='r', linestyle='--', linewidth=1.5, label=f'Average Error: {self.average_error:.3f} m')

        plt.xlabel('Time (s)')
        plt.ylabel('Euclidean Error (m)')
        plt.title('EKF Euclidean Error Over Time')
        plt.legend()

        # Adjust layout and save the plot
        plt.tight_layout()
        plt.savefig('/home/raquel/catkin_ws/src/turtlebot3_datasets/graphs/ekf/ekf_error_plot.png', dpi=300)
        plt.close()

if __name__ == '__main__':
    # Path to the CSV file containing the EKF error data
    csv_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/ekf/ekf_error.csv'

    # Create the EKF error plot object with the loaded data
    ekf_error_plot = EKFErrorPlot(csv_filepath)

    # Plot the EKF error results
    ekf_error_plot.plot_ekf_error()
