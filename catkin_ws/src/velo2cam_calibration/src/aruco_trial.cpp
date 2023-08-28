#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert ROS image message to OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // AprilTag detection parameters
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

        if (!markerIds.empty()) {
            // Draw marker outlines
            cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);

            // Display image with detected markers
            cv::imshow("Detected ArucoTags", image);
            cv::waitKey(1);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_tag_detection");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    ros::spin();

    return 0;
}
