import open3d as o3d
# import math
# import tf.transformations as tf

def main():
    # Replace 'your_point_cloud.pcd' with the path to your PCD file
    pcd_file = '/home/sush/klab2/camera_calib/extended_livox/catkin_ws/src/extended_lidar_camera_calib/floam/PCD/trial.pcd'

    # Load the PCD file using Open3D
    point_cloud = o3d.io.read_point_cloud(pcd_file)

    if not point_cloud:
        print(f"Error: Unable to load {pcd_file}")
        return

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])

def quat_to_euler():
    # Given quaternion components
    quaternion = [-0.5, 0.49999999999755174, -0.5, 0.5000000000024483]

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    euler_angles = tf.euler_from_quaternion(quaternion)

    # Convert radians to degrees
    euler_angles_deg = [math.degrees(angle) for angle in euler_angles]

    print("Euler angles (degrees):", euler_angles_deg)

if __name__ == "__main__":
    # quat_to_euler()
    main()