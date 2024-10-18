import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

class AMCLConfidenceEllipse:
    def __init__(self, csv_filepath: str, time_value: float) -> None:
        # Load data from the CSV file
        self.data = pd.read_csv(csv_filepath)

        # Filter the data to find the entry with the specified time
        selected_data = self.data[self.data['__time'] == time_value]
        if selected_data.empty:
            raise ValueError(f"No data entry found with time = {time_value}")

        # Extract the pose and covariance values
        self.amcl_pose_x = selected_data['/robot_pose_with_covariance_amcl/pose/pose/position/x'].values[0]
        self.amcl_pose_y = selected_data['/robot_pose_with_covariance_amcl/pose/pose/position/y'].values[0]
        self.P_xx = selected_data['/robot_pose_with_covariance_amcl/pose/covariance[0]'].values[0]
        self.P_xy = selected_data['/robot_pose_with_covariance_amcl/pose/covariance[1]'].values[0]
        self.P_yx = selected_data['/robot_pose_with_covariance_amcl/pose/covariance[6]'].values[0]
        self.P_yy = selected_data['/robot_pose_with_covariance_amcl/pose/covariance[7]'].values[0]

        # Covariance matrix
        self.P = np.array([[self.P_xx, self.P_xy],
                           [self.P_yx, self.P_yy]])

        # Plot the confidence ellipse
        fig, ax = plt.subplots()
        self.plot_confidence_ellipse(self.amcl_pose_x, self.amcl_pose_y, self.P, n_std=2.0, ax=ax, edgecolor='red', facecolor='none', label='95% Confidence Ellipse')
        ax.plot(self.amcl_pose_x, self.amcl_pose_y, 'ro', label='AMCL Pose')  # Plot the AMCL pose

        # Set plot limits and labels
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title(f'AMCL Confidence Ellipse at time 76.18s')
        ax.legend()  # Show the legend
        plt.grid()
        plt.savefig('/home/raquel/catkin_ws/src/turtlebot3_datasets/graphs/amcl/amcl_confidence_ellipse.png', dpi=300)
        plt.close()

    def plot_confidence_ellipse(self, x, y, P, n_std=1.0, ax=None, **kwargs):
        """
        Plot a confidence ellipse centered at (x, y) with the covariance matrix P.

        Parameters:
        - x, y: AMCL pose coordinates.
        - P: 2x2 covariance matrix.
        - n_std: Number of standard deviations for the ellipse.
        - ax: Matplotlib Axes object. If None, use the current axis.
        - kwargs: Additional keyword arguments for the Ellipse patch.
        """
        if ax is None:
            ax = plt.gca()

        # Eigenvalue decomposition to get the ellipse parameters
        eigvals, eigvecs = np.linalg.eigh(P)
        # Sort the eigenvalues in descending order
        order = eigvals.argsort()[::-1]
        eigvals, eigvecs = eigvals[order], eigvecs[:, order]

        # Angle of the ellipse (in degrees)
        angle = np.degrees(np.arctan2(*eigvecs[:, 0][::-1]))

        # Width and height of the ellipse
        width, height = 2 * n_std * np.sqrt(eigvals)

        # Create the Ellipse patch
        ellipse = Ellipse((x, y), width, height, angle, **kwargs)

        # Add the ellipse to the plot
        ax.add_patch(ellipse)
        ax.set_aspect('equal')

        # Display the ellipse parameters
        print(f"Ellipse Center: ({x}, {y}), Width: {width}, Height: {height}, Angle: {angle} degrees")

        return ellipse

if __name__ == '__main__':
    # Path to the CSV file containing the AMCL error data
    csv_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/ellipse.csv'
    # Time value for which to plot the ellipse
    time_value = 76.18203401565552

    # Create the AMCL error plot object with the loaded data
    amcl_error_plot = AMCLConfidenceEllipse(csv_filepath, time_value)
