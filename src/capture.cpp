#include<ros/ros.h>
#include<iostream>
#include<chrono>
#include<control_station/RsDataMsg.h>
#include<control_station/RsOperator.h>
#include<opencv2/aruco.hpp>
#include<opencv2/opencv.hpp>
#include<librealsense2/rs.hpp>

bool send_distance(control_station::RsOperator::Request &req ,control_station::RsOperator::Response &res){
    //request_distance.output_distance =  
} 
int main(int argc, char **argv)try{

    ros::init(argc, argv, "realsense_info");
    ros::NodeHandle nh;

    ros::Publisher ros_realsense_pub = nh.advertise<control_station::RsDataMsg>("rs_msg", 1000);
    ros::ServiceServer ros_realsense_srv = nh.advertiseService("rs_srv", send_distance);
    ros::Rate loop_rate(20);

    control_station::RsDataMsg msg;
    control_station::RsOperator srv;

    rs2::pipeline pipe;
    rs2::config cfg;
     cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

     // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();
    while(true){
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();

        cv::Mat color(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
       // マーカーの検出
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(color, dictionary, marker_corners , marker_ids, parameters);

        int sum_marker_coordinate_x = 0;
        int sum_marker_coordinate_y = 0;
        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        if(marker_ids.size() > 0){
            //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            for(int i = 0;i < 4; i++){
                sum_marker_coordinate_x += marker_corners[cv::aruco::DICT_4X4_50][i].x;
                sum_marker_coordinate_y += marker_corners[cv::aruco::DICT_4X4_50][i].y;
                //std::cout << marker_corners[cv::aruco::DICT_4X4_50][0].x << std::endl;
            }
            std::chrono::steady_clock::time_point now_time = std::chrono::steady_clock::now(); 
            std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - previous_time);
            if(elapsed_time.count() > 50){
                int center_marker_x = sum_marker_coordinate_x / 4; 
                int center_marker_y = sum_marker_coordinate_y / 4;  
                previous_time = now_time;
                std::cout << "x:" << center_marker_x << "y:" << center_marker_y << std::endl;
            }
        }
       /*cv::Mat cameraMatrix, distCoeffs;
        std::vector <cv::Vec3d> rvecs,tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners,0.05, cameraMatrix,distCoeffs,rvecs,tvecs);*/
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
}catch(const rs2::error & e){
    std::cerr << "Realsense error calling" << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl; 
    return EXIT_FAILURE;
}catch(const std::exception& e){
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}