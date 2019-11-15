#include <ros/ros.h>
#include <cstdlib>
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
#include <cs_connection/PrintStatus.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cs_connection/RsDataMsg.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <sstream>
#include <string>
#include <fstream>

constexpr std::size_t WIDTH = 1280 / 2;
constexpr std::size_t HEIGHT = 720 / 2;
constexpr double ratio = WIDTH / (double)HEIGHT;
constexpr bool filter = true;
constexpr short hole_fillter_mode = 1;
constexpr int LOGO_X = 350;
constexpr int LOGO_Y = 290;

int orion_status;

struct  realsense_distance
{
    float rs_x;
    float rs_y;
    float rs_z;    
};

struct rgb_data
{
    float x;
    float y;
    float z;
    float yaw;
    float roll;
    float pitch;
};

void sendPicture(std::string filename)
{
    std::string scp("sshpass -p 'Chihayahuru17Ariwara' scp ");
    std::string adr(" tanaka@tanaka-CFSZ5-3L.local:/home/tanaka/2019robocon/src/robot_twitter/img/");
    std::string putcommand = scp + filename + adr;
    const char* cstr = putcommand.c_str();
    std::system(cstr);
}

void statusCallback(const cs_connection::PrintStatus& orion)
{
  orion_status = orion.status;
  std::cout << "orion_status" << std::endl;
}

int main(int argc, char **argv) try
{
    ros::init(argc, argv, "realsense_info");
    ros::NodeHandle nh;

    //image_transport::ImageTransport it(nh);
    //image_transport::Publisher image_pub = it.advertise("image_msg", 10);
    ros::Publisher ros_realsense_pub = nh.advertise<cs_connection::RsDataMsg>("rs_msg", 500);
    ros::Subscriber orion_status_sub = nh.subscribe("print_status", 1000, statusCallback);
    ros::Rate loop_rate(100);

    ROS_INFO("Started control station");
    cs_connection::RsDataMsg rs_msg;

    rs2::colorizer cr;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);
    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2::hole_filling_filter hole_filling(hole_fillter_mode);
    rs2::spatial_filter spat_filling;
    // rs2::align align_to_depth(RS2_STREAM_DEPTH);

    auto intr = depth_stream.get_intrinsics();
    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);
    std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();

    // cv::Mat cameraMatrix, distCoeffs;
    std::array<std::array<float ,4>,5> distance_save;

    struct realsense_distance rdist;
    struct rgb_data rgb;
    //short count = 0;
    short start_count = 0;
    int numbering = 0;
    //std::time_t
    std::ostringstream video_name, record_name;
    auto real_time = std::chrono::system_clock::now();
    std::time_t sys_time = std::chrono::system_clock::to_time_t(real_time);

    std::ofstream log;
    record_name << std::ctime(&sys_time) << ".csv" << std::flush;
    const std::string tmp_name = record_name.str();
    const char *char_to_record_name = tmp_name.c_str();
    log.open(char_to_record_name);

    video_name << std::ctime(&sys_time) << ".mp4" << std::flush;
    cv::VideoWriter::fourcc('M', 'P', '4', 'S');
    cv::VideoWriter writer(video_name.str(), cv::VideoWriter::fourcc('M', 'P', '4', 'S'), 30.0, cv::Size(WIDTH, HEIGHT));
   
    //cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 465.33068596, 0, 321.98956352, 0, 464.52222509, 164.37789961, 0, 0, 1);    

    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 465.33068563,0,321.98961202,0,464.52224766 ,164.37790344,0,0,1);    
    //cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.07103586, 0.13770576, -0.00838434, -0.00454973, -0.81692506);

    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.07103579,  0.13770855, -0.00838435, -0.00454967, -0.81693468);

    cv::Mat roboken_logo = cv::imread("/home/yusuke/roboken.jpg",1);
    cv::resize(roboken_logo,roboken_logo,cv::Size(),0.1,0.1);    

    bool start_orion = false;
    bool bathtowel_finish = false;
    bool sheets_finish = false;
    bool arrival_home = false;

    while (true)
    {
        float dist_left_base, dist_right_base;

        rs2::frameset frames = pipe.wait_for_frames();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        auto aligned_frames = align_to_color.process(frames);
        auto depth_map = aligned_frames.get_depth_frame();
        auto color_map = aligned_frames.get_color_frame();
        
        if (filter)
        {
            depth_map = hole_filling.process(depth_map);
            depth_map = spat_filling.process(depth_map);
        }

        cv::Mat color(cv::Size(color_map.get_width(), color_map.get_height()), CV_8UC3, (void *)color_map.get_data(), cv::Mat::AUTO_STEP);
        //cv::cvtColor(color, color,CV_RGB2GRAY);

        dist_left_base = depth_map.get_distance(280 / 2, 530 / 2);
        dist_right_base = depth_map.get_distance(1000 / 2, 530 / 2);

        float diff_base_distance = dist_left_base - dist_right_base;
        cv::line(color, cv::Point(280 / 2, 530 / 2), cv::Point(280 / 2, 530 / 2), cv::Scalar(0, 255, 100), 10, 16);
        cv::line(color, cv::Point(1000 / 2, 530 / 2), cv::Point(1000 / 2, 530 / 2), cv::Scalar(255, 225, 100), 10, 16);
        cv::line(color, cv::Point(340 / 2, 216 / 2), cv::Point(940 / 2, 216 / 2), cv::Scalar(255, 0, 0), 5, 16);
        cv::line(color, cv::Point(613, 0), cv::Point(620, 290), cv::Scalar(255, 0, 0), 5, 16);
        cv::line(color, cv::Point(25, 0), cv::Point(18, 290), cv::Scalar(255, 0, 0), 5, 16);
        cv::line(color, cv::Point(50, 280), cv::Point(50, 335), cv::Scalar(255, 255, 0), 1, 4);
        cv::line(color, cv::Point(590, 280), cv::Point(590, 335), cv::Scalar(255, 255, 0), 1, 4);
        /*if(diff_base_distance < 1){
            std::cout << diff_base_distance * 1000 << std::endl;
        }*/
        std::cout << dist_left_base << std::endl;
        //std::cout << dist_right_base << std::endl;

        cv::imshow("set", color);
        //depth_map.get_distance()
        if (cv::waitKey(5) == 116)
        {
            cv::destroyAllWindows();
            break;
        }
    }

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
        if (filter)
        {
            depth_map = hole_filling.process(depth_map);
            //depth_map = spat_filling.process(depth_map);
        }
        // auto colorized_depth = cr.colorize(depth_map);

        cv::Mat color(cv::Size(color_map.get_width(), color_map.get_height()), CV_8UC3, (void *)color_map.get_data(), cv::Mat::AUTO_STEP);
        //        cv::Mat depth(cv::Size(depth_map.get_width(), depth_map.get_height()), CV_8UC3, (void *)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
        //cv::cvtColor(color, color,CV_RGB2GRAY);
        // マーカーの検出
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(color, dictionary, marker_corners, marker_ids, parameters);
        //cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.05, cameraMatrix, distCoeffs, rvecs,tvecs);
        cv::aruco::drawDetectedMarkers(color, marker_corners, marker_ids);
        
        //--------------------pose estimation-------------------------------------//
        //        std::vector< cv::Vec3d > rvecs, tvecs;
        //        cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        //----------------------------------------------------------------------------//
        
        float sum_marker_coordinate_x = 0;
        float sum_marker_coordinate_z = 0;
        float marker_coodinate_y = 0;
        bool exist = false;

        if (marker_ids.size() > 0 && marker_ids.size() < 4)
        {
            for (int marker_num_count = 0; marker_num_count < marker_ids.size(); marker_num_count++)
            {
                if (marker_ids.at(marker_num_count) == 0)
                {
                    exist = true;
                    for (int i = 0; i < 4; i++)
                    {
                        sum_marker_coordinate_x += marker_corners[marker_num_count][i].x;
                        sum_marker_coordinate_z += marker_corners[marker_num_count][i].y;
                    }

                    cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.406, cameraMatrix, distCoeffs, rvecs,tvecs);
                    rgb.x = tvecs[marker_num_count].val[0];
                    rgb.y = tvecs[marker_num_count].val[2];
                    rgb.z = tvecs[marker_num_count].val[1];
                    rgb.yaw   = rvecs[marker_num_count].val[2] * 180 / M_PI;
                    rgb.roll  = rvecs[marker_num_count].val[0] * 180 / M_PI;
                    rgb.pitch = rvecs[marker_num_count].val[1] * 180 / M_PI;
                    cv::aruco::drawAxis(color, cameraMatrix, distCoeffs, rvecs[marker_num_count], tvecs[marker_num_count], 0.1);

                    //std::cout << rgb.x  << "   " << rgb.y << "   " << rgb.z << std::endl;
                    //std::cout << rgb.yaw << "   " << rgb.roll << "   " << rgb.pitch << std::endl;
                    std::chrono::steady_clock::time_point now_time = std::chrono::steady_clock::now();
                    std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - previous_time);
                    if (elapsed_time.count() > 1)
                    {
                        float point_center_marker_x = sum_marker_coordinate_x / 4;
                        float point_center_marker_z = sum_marker_coordinate_z / 4;

                        std::array<float,4> marker_info_average;
                        float depth_average = (depth_map.get_distance(point_center_marker_x, point_center_marker_z) + depth_map.get_distance(point_center_marker_x + 1, point_center_marker_z) + depth_map.get_distance(point_center_marker_x - 1, point_center_marker_z) + depth_map.get_distance(point_center_marker_x, point_center_marker_z - 1) + depth_map.get_distance(point_center_marker_x, point_center_marker_z + 1)) / 5.0;
                        /*distance_save.at(4)[0] = depth_average;
                        distance_save.at(4)[1] = rgb.x;
                        distance_save.at(4)[2] = rgb.y;
                        distance_save.at(4)[3] = rgb.z;*/
                        distance_save.at(4) = {depth_average,rgb.x,rgb.y,rgb.z};

                        //std::cout << " qqq" << std::endl;
                        start_count++;
                        if (start_count > 8)
                        {
                            for(int k = 0; k < 4; k++){
                                marker_info_average.at(k) = (distance_save.at(0)[k]  + (distance_save.at(1)[k] * 0.4 + distance_save.at(2)[k] * 0.3 + distance_save.at(3)[k] * 0.2 + distance_save.at(4)[k] * 0.1)) / 2;
                            }
                            depth_average = marker_info_average.at(0);
                            rgb.x = marker_info_average.at(1);
                            rgb.y = marker_info_average.at(2);
                            rgb.z = marker_info_average.at(3);

                            start_count = 8;
                            float point[3] = {0, 0, 0};
                            float pixel[2] = {0, 0};
                            pixel[0] = point_center_marker_x;
                            pixel[1] = point_center_marker_z;
                            rs2_deproject_pixel_to_point(point, &intr, pixel, depth_average);
                            //double distance_sum = marker_distance + distance_sum;
                            previous_time = now_time;
                            rdist.rs_x = (point[0] * 1000 + rgb.x * 1000) / 2;
                            rdist.rs_y = (point[2] * 1000 + rgb.y * 1000) / 2;
                            rdist.rs_z = (point[1] * 1000 + rgb.z * 1000) / 2;

                            log << rdist.rs_x << "," << rdist.rs_y << "," << rdist.rs_z << std::endl;
                            //ROS_INFO("x:%f  y:%f  z:%f ", rdist.rs_x, rdist.rs_y, rdist.rs_z);
                            rs_msg.x_distance = rdist.rs_x;
                            rs_msg.y_distance = rdist.rs_y;
                            rs_msg.z_distance = rdist.rs_z;
                            ros_realsense_pub.publish(rs_msg);
                        }
                    }
                }
            }
            
            if (exist == false)
            {
                float center_marker_x = -50000;
                float center_marker_y = -50000;
                float center_marker_z = -50000;
                rs_msg.x_distance = -50000;
                rs_msg.y_distance = -50000;
                rs_msg.z_distance = -50000;
                //std::cout << center_marker_x << std::endl;
                ros_realsense_pub.publish(rs_msg);
                exist = false;
            }
        }
        else
        {
            float center_marker_x = -50000;
            float center_marker_y = -50000;
            float center_marker_z = -50000;
            rs_msg.x_distance = -50000;
            rs_msg.y_distance = -50000;
            rs_msg.z_distance = -50000;
            //std::cout << center_marker_x << std::endl;
            ros_realsense_pub.publish(rs_msg);
        }

        // 検出したマーカー

       
        cv::Mat line_in = color;
        // cv::line(line_in,cv::Point(340/2,215/2),cv::Point(940/2,215/2), cv::Scalar(255,0,100), 5, 16);

        cv::imshow("marker_detection", line_in);
        //        cv::imshow("cr",depth);
        writer << color;

        switch(orion_status){
            case 1:
                start_orion = true;
                break;
            case 7:
                bathtowel_finish = true;
                break;
            case 15:
                sheets_finish = true;
                break;
            case 21:
                arrival_home = true;
                break;
            default:
                start_orion = false;
                bathtowel_finish = false;
                sheets_finish = false;
                arrival_home = false;
                break;
        }

        cv::Mat merge;

        if(start_orion == true){
            merge = color(cv::Rect(LOGO_X,LOGO_Y,roboken_logo.cols,roboken_logo.rows));
            roboken_logo.copyTo(merge);
            cv::imwrite("start.jpg",color);
            //std::cout << "send" << std::endl;
            sendPicture("start.jpg");
            //sensor_msgs::ImagePtr picture_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", color).toImageMsg();
            start_orion = false;            
            //image_pub.publish(picture_msg);
        }else if(bathtowel_finish == true){
            merge = color(cv::Rect(LOGO_X,LOGO_Y,roboken_logo.cols,roboken_logo.rows));
            roboken_logo.copyTo(merge);
            cv::imwrite("bathtowel.jpg",color);
            sendPicture("bathtowel.jpg");
            //sensor_msgs::ImagePtr picture_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", color).toImageMsg();
            //std::cout << "send" << std::endl;
            bathtowel_finish = false;            
            //image_pub.publish(picture_msg);
        }else if(sheets_finish == true){
            merge = color(cv::Rect(LOGO_X,LOGO_Y,roboken_logo.cols,roboken_logo.rows));
            roboken_logo.copyTo(merge);
            cv::imwrite("sheets.jpg",color);
            sendPicture("sheets.jpg");
            //sensor_msgs::ImagePtr picture_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", color).toImageMsg();
            //std::cout << "send" << std::endl;
            sheets_finish = false;            
            //image_pub.publish(picture_msg);
        }else if(arrival_home == true){
            merge = color(cv::Rect(LOGO_X,LOGO_Y,roboken_logo.cols,roboken_logo.rows));
            roboken_logo.copyTo(merge);
            cv::imwrite("arrival.jpg",color);
            sendPicture("arrival.jpg");
            //sensor_msgs::ImagePtr picture_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", color).toImageMsg();
            //std::cout << "send" << std::endl;
            arrival_home = false;            
            //image_pub.publish(picture_msg);
        }

        //sensor_msgs::ImagePtr picture_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", color).toImageMsg();
        int key = cv::waitKey(10);
        if (key == 115)
        {
            cv::imwrite("data.jpg", color);
            std::cout << "cap" << std::endl;
            numbering++;
            std::ostringstream picture_name;
            picture_name << "data" << numbering << ".jpg" << std::flush;
            cv::imwrite(picture_name.str(), color);
        }
        else if (key == 27)
        {
            break;
        }

        /* count = count + 1;
        if(count > 3){
            count = 0;
        }*/

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
