#include<ros/ros.h>
#include<iostream>
#include<sstream>
#include<control_station/RsDataMsg.h>
#include<control_station/RsOperator.h>
#include<opencv2/aruco.hpp>
#include<opencv2/opencv.hpp>
#include<librealsense2/rs.hpp>

bool send_distance(control_station::RsOperator::Request &req ,control_station::RsOperator::Response &res){
    //request_distance.output_distance =  
} 


int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_info");
    ros::NodeHandle nh;

    ros::Publisher ros_realsense_pub = nh.advertise<control_station::RsDataMsg>("rs_msg", 1000);
    ros::ServiceServer ros_realsense_srv = nh.advertiseService("rs_srv", send_distance);
    ros::Rate loop_rate(30);

    control_station::RsDataMsg msg;
    control_station::RsOperator srv;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

     // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);


    //const auto window_name = "RealSense Image";
    //namedWindow(window_name, WINDOW_AUTOSIZE);
    //double pixel_distance_in_meters;

    while(true){
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();

        cv::Mat color(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);


       // マーカーの検出
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(color, dictionary, marker_corners, marker_ids, parameters);

        // 検出したマーカーの描画
        cv::aruco::drawDetectedMarkers(color, marker_corners, marker_ids);
        cv::imshow("marker_detection", color);
        //cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
        if (cv::waitKey(10) == 27){
            break;
        }
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}