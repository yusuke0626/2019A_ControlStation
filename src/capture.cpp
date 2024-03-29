#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cs_connection/RsDataMsg.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <sstream>
#include <fstream>

constexpr std::size_t WIDTH = 1280;
constexpr std::size_t HEIGHT = 720;
constexpr double ratio = WIDTH / (double)HEIGHT;

int main(int argc, char **argv) try
{

    ros::init(argc, argv, "realsense_info");
    ros::NodeHandle nh;

    ros::Publisher ros_realsense_pub = nh.advertise<cs_connection::RsDataMsg>("rs_msg", 1000);
    ros::Rate loop_rate(100);

    ROS_INFO("Started control station");

    cs_connection::RsDataMsg rs_msg;

    //rs2::colorizer cr;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);
    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    // rs2::align align_to_depth(RS2_STREAM_DEPTH);

    auto intr = depth_stream.get_intrinsics();

    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);
    std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();

   // cv::Mat cameraMatrix, distCoeffs;
    
    float distance_save[5] {0,0,0,0,0};
    short count = 0;
    short start_count = 0;
    int numbering = 0;
    //std::time_t 
    std::ostringstream video_name,record_name;
    auto real_time = std::chrono::system_clock::now(); 
    std::time_t sys_time = std::chrono::system_clock::to_time_t(real_time);
    
    std::ofstream log;
    record_name << std::ctime(&sys_time) << ".csv" << std::flush;
    const std::string tmp_name = record_name.str();
    const char* char_to_record_name = tmp_name.c_str();
    log.open(char_to_record_name);

    video_name << std::ctime(&sys_time) << ".mp4" << std::flush;
    cv::VideoWriter::fourcc('M', 'P', '4', 'S');
    cv::VideoWriter writer(video_name.str(),cv::VideoWriter::fourcc('M', 'P', '4', 'S'), 8.0, cv::Size(WIDTH,HEIGHT));

    while (true)
    {
        
        start_count = start_count + 1;
        distance_save[0] = distance_save[1];
        distance_save[1] = distance_save[2];
        distance_save[2] = distance_save[3];
        distance_save[3] = distance_save[4];
        
        rs2::frameset frames = pipe.wait_for_frames();
        //rs2::depth_frame depth_point = frames.get_depth_frame;
        rs2::align align_to_color(RS2_STREAM_COLOR);
        auto aligned_frames = align_to_color.process(frames);
        auto depth_map = aligned_frames.get_depth_frame();
        auto color_map = aligned_frames.get_color_frame();
//        auto colorized_depth = cr.colorize(depth_map);

        cv::Mat color(cv::Size(color_map.get_width(), color_map.get_height()), CV_8UC3, (void *)color_map.get_data(), cv::Mat::AUTO_STEP);
//        cv::Mat depth(cv::Size(depth_map.get_width(), depth_map.get_height()), CV_8UC3, (void *)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
        // マーカーの検出
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(color, dictionary, marker_corners, marker_ids, parameters);
//        std::vector< cv::Vec3d > rvecs, tvecs;
//        cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

        float sum_marker_coordinate_x = 0;
        float sum_marker_coordinate_z = 0;
        float marker_coodinate_y = 0;

        if (marker_ids.size() > 0 && marker_ids.size() < 4)
        {
            if (marker_ids.at(0) == 0)
            {
                for (int i = 0; i < 4; i++)
                {
                    sum_marker_coordinate_x += marker_corners[0][i].x;
                    sum_marker_coordinate_z += marker_corners[0][i].y;
                }

                std::chrono::steady_clock::time_point now_time = std::chrono::steady_clock::now();
                std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - previous_time);
                if (elapsed_time.count() > 15)
                {
                    float point_center_marker_x = sum_marker_coordinate_x / 4;
                    float point_center_marker_z = sum_marker_coordinate_z / 4;

                    float marker_distance = 0;

                    distance_save[4] = (depth_map.get_distance(point_center_marker_x, point_center_marker_z) + depth_map.get_distance(point_center_marker_x + 1, point_center_marker_z) + depth_map.get_distance(point_center_marker_x - 1, point_center_marker_z) + depth_map.get_distance(point_center_marker_x, point_center_marker_z - 1) + depth_map.get_distance(point_center_marker_x, point_center_marker_z + 1))  / 5.0;
                    start_count++;                    
                    if(start_count > 8){
                        marker_distance = (distance_save[0] + distance_save[1] + distance_save[2] + distance_save[3] + distance_save[4]) / 5;
                        start_count = 8;
                        float point[3] = {0, 0, 0};
                        float pixel[2] = {0, 0};
                        pixel[0] = point_center_marker_x;
                        pixel[1] = point_center_marker_z;
                        rs2_deproject_pixel_to_point(point, &intr, pixel, marker_distance);
                        //double distance_sum = marker_distance + distance_sum;
                        previous_time = now_time;
                        float center_marker_x = point[0] * 1000; 
                        float center_marker_y = point[2] * 1000; 
                        float center_marker_z = point[1] * 1000; 
                        log << center_marker_x << "," << center_marker_y << "," << center_marker_z << std::endl;
                        ROS_INFO("x:%f  y:%f  z:%f   d:%f", center_marker_x, center_marker_y, center_marker_z, marker_distance);
                        rs_msg.x_distance = center_marker_x;
                        rs_msg.y_distance = center_marker_y;
                        rs_msg.z_distance = center_marker_z;
                        ros_realsense_pub.publish(rs_msg);
                    }
                }
            }
        }

        // 検出したマーカーの描画
//        cv::aruco::drawDetectedMarkers(color, marker_corners, marker_ids);
       // cv::aruco::drawAxis(color, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
        cv::Mat line_in = color;
        cv::line(line_in,cv::Point(340,240),cv::Point(940,240), cv::Scalar(255,100,100), 5, 16);
                
        cv::imshow("marker_detection", line_in);
        writer << color;
        int key = cv::waitKey(10); 
        if(key == 115){
            //cv::imwrite("data.png", color);
            std::cout << "cap" << std::endl;
            numbering++;
            std::ostringstream picture_name;
            picture_name << "data" << numbering  << ".png "<< std::flush;
            cv::imwrite(picture_name.str(), color);
        }else if (key == 27){
            break;
        }   
        
        count = count + 1;
        if(count > 5){
            count = 0;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    log.close();
    return 0;
}
catch (const rs2::error &e)
{
    std::cerr << "Realsense error calling" << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}