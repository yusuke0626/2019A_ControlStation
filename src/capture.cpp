#include<ros/ros.h>
#include<iostream>
#include<chrono>
#include<control_station/RsDataMsg.h>
#include<control_station/RsOperator.h>
#include<opencv2/aruco.hpp>
#include<opencv2/opencv.hpp>
#include<librealsense2/rs.hpp>

constexpr std::size_t WIDTH = 1280;
constexpr std::size_t HEIGHT = 720;
constexpr double ratio = WIDTH / (double)HEIGHT;

int main(int argc, char **argv)try{

    ros::init(argc, argv, "realsense_info");
    ros::NodeHandle nh;

    ros::Publisher ros_realsense_pub = nh.advertise<control_station::RsDataMsg>("rs_msg", 1000);
    ros::Rate loop_rate(100);

    control_station::RsDataMsg rs_msg;

    rs2::colorizer cr;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    //rs2::align align_to_depth(RS2_STREAM_DEPTH);
    
    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::align align_to_depth(RS2_STREAM_DEPTH);

     // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();
    while(true){
        rs2::frameset frames = pipe.wait_for_frames();
        //rs2::depth_frame depth_point = frames.get_depth_frame;
        auto aligned_frames = align_to_depth.process(frames);

        auto depth_map = aligned_frames.get_depth_frame();
        auto color_map = aligned_frames.get_color_frame();
        auto colorized_depth = cr.colorize(depth_map);

        cv::Mat color(cv::Size(color_map.get_width(),color_map.get_height()), CV_8UC3, (void*)color_map.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(depth_map.get_width(),depth_map.get_height()), CV_8UC3, (void*)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
       // マーカーの検出
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(color, dictionary, marker_corners , marker_ids, parameters);

        int sum_marker_coordinate_x = 0;
        int sum_marker_coordinate_z = 0;

        if(marker_ids.size() > 0){
            for(int i = 0;i < 4; i++){
                sum_marker_coordinate_x += marker_corners[cv::aruco::DICT_4X4_50][i].x;
                sum_marker_coordinate_z += marker_corners[cv::aruco::DICT_4X4_50][i].y;
            }
            std::chrono::steady_clock::time_point now_time = std::chrono::steady_clock::now(); 
            std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - previous_time);
            if(elapsed_time.count() > 15){
                int center_marker_x = sum_marker_coordinate_x / 4; 
                int center_marker_z = sum_marker_coordinate_z / 4;  
                double center_marker_y = depth_map.get_distance(center_marker_x,center_marker_z);
                previous_time = now_time;
                std::cout << "x:" << center_marker_x << "  z:" << center_marker_z << "  y:" << center_marker_y << std::endl;
                rs_msg.x_distance = center_marker_x;
                rs_msg.y_distance = center_marker_y;
                rs_msg.z_distance = center_marker_z;
                ros_realsense_pub.publish(rs_msg);
            }
        }

        // 検出したマーカーの描画
        cv::aruco::drawDetectedMarkers(color, marker_corners, marker_ids);
        //cv::imshow("marker_detection", color);
        //cv::imshow("depth",depth);
        cv::Mat dst;
        cv::addWeighted(color, 0.9, depth, 0.1, 0.0, dst);
        cv::imshow("merge",dst);

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
