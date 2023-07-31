import open3d as o3d

def main():
    # Replace 'your_point_cloud.pcd' with the path to your PCD file
    pcd_file = '/home/sush/klab2/camera_calib/extended_livox/catkin_ws/src/extended_lidar_camera_calib/floam/PCD/map_.pcd'

    # Load the PCD file using Open3D
    point_cloud = o3d.io.read_point_cloud(pcd_file)

    if not point_cloud:
        print(f"Error: Unable to load {pcd_file}")
        return

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    main()