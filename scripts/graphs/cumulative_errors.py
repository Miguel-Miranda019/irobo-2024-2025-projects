import pandas as pd
import matplotlib.pyplot as plt

class CumulativeErrorPlot:
    def __init__(self, csv_filepath):
        # Load the merged data from the CSV file
        self.data = pd.read_csv(csv_filepath)

        # Extract relevant columns and convert them to 1D arrays
        self.times = self.data['__time'].values
        self.cumulative_ekf_error = self.data['cumulative_ekf_error'].values
        self.cumulative_amcl_error = self.data['cumulative_amcl_error'].values

        # Print the original cumulative errors for debugging
        print("Original Cumulative EKF Error:", self.cumulative_ekf_error)
        print("Original Cumulative AMCL Error:", self.cumulative_amcl_error)

        # Handle NaN values through interpolation
        self.cumulative_ekf_error = self.interpolate_values(self.cumulative_ekf_error)
        self.cumulative_amcl_error = self.interpolate_values(self.cumulative_amcl_error)

    def interpolate_values(self, data):
        # Convert to a Pandas Series to use interpolation method
        return pd.Series(data).interpolate().fillna(method='bfill').values

    def plot_cumulative_errors(self):
        plt.figure(figsize=(10, 6))

        # Plotting both cumulative EKF and AMCL errors
        plt.plot(self.times, self.cumulative_ekf_error, label='Cumulative EKF Error', color='blue', linewidth=2)
        plt.plot(self.times, self.cumulative_amcl_error, label='Cumulative AMCL Error', color='orange', linewidth=2)

        # Adding labels and title
        plt.xlabel('Time (s)')
        plt.ylabel('Cumulative Error')
        plt.title('Cumulative EKF vs. AMCL Error Over Time')
        plt.legend()

        # Start Y-axis at 0
        plt.ylim(bottom=0)

        # Show grid
        plt.grid(True)

        # Save the plot as a PNG file
        plt.savefig('/home/raquel/catkin_ws/src/turtlebot3_datasets/graphs/cumulative_error_plot.png', dpi=300)


if __name__ == '__main__':
    # Path to the merged CSV file
    csv_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/merged_errors.csv'

    # Create the plot object and plot the cumulative errors
    cumulative_error_plot = CumulativeErrorPlot(csv_filepath)
    cumulative_error_plot.plot_cumulative_errors()
