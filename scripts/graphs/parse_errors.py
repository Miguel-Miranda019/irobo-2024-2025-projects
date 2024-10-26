import pandas as pd

def merge_error_data(ekf_filepath, amcl_filepath, output_filepath):
    # Load the EKF error data
    ekf_data = pd.read_csv(ekf_filepath, usecols=['__time', '/ekf_error/data'])
    
    # Load the AMCL error data
    amcl_data = pd.read_csv(amcl_filepath, usecols=['__time', '/amcl_error/data'])
    
    # Merge the two DataFrames on the __time column with outer join
    merged_data = pd.merge(ekf_data, amcl_data, on='__time', how='outer', suffixes=('_ekf', '_amcl'))
    
    # Handle duplicate __time values (if any)
    merged_data = merged_data.groupby('__time').agg({
        '/ekf_error/data': 'first',  # Choose the first available value for EKF
        '/amcl_error/data': 'first'   # Choose the first available value for AMCL
    }).reset_index()
    
    # Calculate cumulative errors
    merged_data['cumulative_ekf_error'] = merged_data['/ekf_error/data'].cumsum()
    merged_data['cumulative_amcl_error'] = merged_data['/amcl_error/data'].cumsum()

    # Save the merged DataFrame to a new CSV file
    merged_data.to_csv(output_filepath, index=False)
    print(f"Merged data saved to '{output_filepath}'.")

if __name__ == '__main__':
    # Paths to the CSV files
    ekf_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/ekf/ekf_error.csv'
    amcl_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/amcl_error.csv'
    output_filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/merged_errors.csv'

    # Call the merge function
    merge_error_data(ekf_filepath, amcl_filepath, output_filepath)
