import pandas as pd
import matplotlib.pyplot as plt

class AMCLErrorPlot:
    def __init__(self, csv_filepath) -> None:
        # Load data from the CSV file
        self.data = pd.read_csv(csv_filepath)

        # Extract relevant columns
        self.times = self.data['__time'].values
        self.amcl_error = self.data['/amcl_error/data'].values

        # Calculate the average error
        self.average_error = self.amcl_error.mean()

        # Print the length of each data array to verify
        print(f"Number of data points: {len(self.times)}")
        print(f"Average AMCL Euclidean Error: {self.average_error:.3f} m")

    def plot_amcl_error(self):
        plt.figure()

        # Plotting the AMCL Euclidean error over time
        plt.plot(self.times, self.amcl_error, label='AMCL Euclidean Error', color='b')

        # Plotting the average error as a horizontal line
        plt.axhline(y=self.average_error, color='r', linestyle='--', linewidth=1.5, label=f'Average Error: {self.average_error:.3f} m')

        plt.xlabel('Time (s)')
        plt.ylabel('Euclidean Error (m)')
        plt.title('AMCL Euclidean Error Over Time')
        plt.legend()

        # Adjust layout and save the plot
        plt.tight_layout()
        plt.savefig('/home/raquel/catkin_ws/src/turtlebot3_datasets/graphs/amcl/amcl_error_plot.png', dpi=300)
        plt.close()

if __name__ == '__main__':
    # Path to the CSV file containing the AMCL error data
    csv_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/amcl_error.csv'

    # Create the AMCL error plot object with the loaded data
    amcl_error_plot = AMCLErrorPlot(csv_filepath)

    # Plot the AMCL error results
    amcl_error_plot.plot_amcl_error()
