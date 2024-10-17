import pandas as pd
import matplotlib.pyplot as plt

class ErrorOverTimePlot:
    def __init__(self, csv_filepath):
        # Load the merged data from the CSV file
        self.data = pd.read_csv(csv_filepath)

        # Extract relevant columns and convert them to 1D arrays
        self.times = self.data['__time'].values
        self.ekf_error = self.data['/ekf_error/data'].values  # Update with the actual error column name
        self.amcl_error = self.data['/amcl_error/data'].values  # Update with the actual error column name

        # Print the original errors for debugging
        print("Original EKF Error:", self.ekf_error)
        print("Original AMCL Error:", self.amcl_error)

        # Handle NaN values through interpolation
        self.ekf_error = self.interpolate_values(self.ekf_error)
        self.amcl_error = self.interpolate_values(self.amcl_error)

    def interpolate_values(self, data):
        # Convert to a Pandas Series to use interpolation method
        return pd.Series(data).interpolate().fillna(method='bfill').values

    def plot_errors_over_time(self):
        plt.figure(figsize=(10, 6))

        # Plotting both EKF and AMCL errors
        plt.plot(self.times, self.ekf_error, label='EKF Error', color='blue', linewidth=2)
        plt.plot(self.times, self.amcl_error, label='AMCL Error', color='orange', linewidth=2)

        # Adding labels and title
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.title('EKF vs. AMCL Error Over Time')
        plt.legend()

        # Set Y-axis limits if necessary
        plt.ylim(bottom=0)  # Start Y-axis at 0 to ensure visibility

        # Show grid
        plt.grid(True)

        # Save the plot as a PNG file
        plt.savefig('/home/raquel/catkin_ws/src/turtlebot3_datasets/graphs/error_over_time_plot.png', dpi=300)

if __name__ == '__main__':
    # Path to the merged CSV file
    csv_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/merged_errors.csv'  # Update with the actual path

    # Create the plot object and plot the errors over time
    error_over_time_plot = ErrorOverTimePlot(csv_filepath)
    error_over_time_plot.plot_errors_over_time()
