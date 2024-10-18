import pandas as pd

# Load the CSV file
input_file = "/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/amcl.csv"
output_file = "/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/filtered_amcl.csv"

# List of columns to keep
columns_to_keep = [
    "__time",
    "/robot_pose_with_covariance_amcl/pose/covariance[0]",
    "/robot_pose_with_covariance_amcl/pose/covariance[1]",
    "/robot_pose_with_covariance_amcl/pose/covariance[6]",
    "/robot_pose_with_covariance_amcl/pose/covariance[7]",
    "/amcl_error/data",
    "/robot_pose_with_covariance_amcl/pose/pose/position/x",
    "/robot_pose_with_covariance_amcl/pose/pose/position/y",
    "/gtInfo/pose/pose/position/x",
    "/gtInfo/pose/pose/position/y"
]

# Read the CSV file and filter columns
try:
    df = pd.read_csv(input_file, usecols=columns_to_keep)

    # Constant value to subtract from the __time column
    time_offset = df["__time"][0]

    # Subtract the offset from all entries in the __time column
    df["__time"] = df["__time"] - time_offset

    # Save the filtered dataframe to a new CSV file
    df.to_csv(output_file, index=False)
    print(f"Filtered CSV saved as '{output_file}'.")

    # Create additional CSV files


    # amcl_error.csv: with "__time" and "/amcl_error/data"
    amcl_error_df = df[["__time", "/amcl_error/data"]].dropna()
    amcl_error_df.to_csv("/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/amcl_error.csv", index=False)
    print("amcl_error.csv saved.")

    # gt.csv: with "__time", "/gtInfo/pose/pose/position/x", 
    #          "/gtInfo/pose/pose/position/y", 
    #          "/robot_pose_with_covariance_amcl/pose/pose/position/x", 
    #          "/robot_pose_with_covariance_amcl/pose/pose/position/y"
    gt_df = df[[
        "__time", 
        "/gtInfo/pose/pose/position/x", 
        "/gtInfo/pose/pose/position/y", 
        "/robot_pose_with_covariance_amcl/pose/pose/position/x", 
        "/robot_pose_with_covariance_amcl/pose/pose/position/y"
    ]]

    # Filter out rows where only __time has values (i.e., where all other columns are NaN)
    gt_df = gt_df[~(gt_df[["/gtInfo/pose/pose/position/x", "/gtInfo/pose/pose/position/y", 
                            "/robot_pose_with_covariance_amcl/pose/pose/position/x", 
                            "/robot_pose_with_covariance_amcl/pose/pose/position/y"]].isna().all(axis=1))]

    gt_df.to_csv("/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/gt.csv", index=False)
    print("gt.csv saved.")

    # ellipse.csv: with "__time" and covariance and positions
    ellipse_df = df[["__time", "/robot_pose_with_covariance_amcl/pose/covariance[0]", "/robot_pose_with_covariance_amcl/pose/covariance[1]", "/robot_pose_with_covariance_amcl/pose/covariance[6]", "/robot_pose_with_covariance_amcl/pose/covariance[7]", "/robot_pose_with_covariance_amcl/pose/pose/position/x", "/robot_pose_with_covariance_amcl/pose/pose/position/y"]].dropna()
    ellipse_df.to_csv("/home/raquel/catkin_ws/src/turtlebot3_datasets/data/amcl/ellipse.csv", index=False)
    print("ellipse.csv saved.")
except ValueError as e:
    print(f"Error: {e}. Please check if the specified columns exist in the CSV file.")

