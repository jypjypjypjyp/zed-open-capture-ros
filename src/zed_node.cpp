#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#define VIDEO_MOD_AVAILABLE 1
#define SENSORS_MOD_AVAILABLE 1

#include <zed-open-capture/sensorcapture.hpp>
#include <zed-open-capture/videocapture.hpp>

class StereoCamera
{
public:
    StereoCamera(sl_oc::video::RESOLUTION resolution, sl_oc::video::FPS fps)
    {
        sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;
        sl_oc::video::VideoParams params;
        params.res = resolution;
        params.fps = fps;
        params.verbose = verbose;

        // Create a Video Capture object
        auto camera_ = new sl_oc::video::VideoCapture(params);
        if (!camera_->initializeVideo(-1))
        {
            std::cerr << "Cannot open camera video capture" << std::endl;
            return;
        }
        else
        {
            camera = camera_;
        }

        // Serial number of the connected camera
        int camSn = camera->getSerialNumber();
        std::cout << "Video Capture connected to camera sn: " << camSn << std::endl;

        // Create a Sensors Capture object
        auto sensor_ = new sl_oc::sensors::SensorCapture(verbose);
        if (!sensor_->initializeSensors(camSn)) // Note: we use the serial number acquired by the VideoCapture object
        {
            std::cerr << "Cannot open sensors capture" << std::endl;
            return;
        }
        else
        {
            sensor = sensor_;
            camera->enableSensorSync(sensor);
        }
    }

    ~StereoCamera()
    {
        delete camera;
        delete sensor;
    }

    // Sensor acquisition runs at 400Hz, so it must be executed in a different thread
    const sl_oc::sensors::data::Imu getLastIMUData()
    {
        return sensor->getLastIMUData();
    }

    const sl_oc::video::Frame getLastFrame()
    {
        return camera->getLastFrame(1);
    }

    sl_oc::video::VideoCapture *camera;
    sl_oc::sensors::SensorCapture *sensor;
};

image_transport::Publisher left_image_pub;
image_transport::Publisher right_image_pub;
ros::Publisher sensor_pub;
sl_oc::video::RESOLUTION gResolution;
sl_oc::video::FPS gFps;
StereoCamera *zed;

void image_callback(const ros::TimerEvent &timer_event)
{
    static double last_timestamp = 0;
    const sl_oc::video::Frame frame = zed->getLastFrame();
    if (frame.data != nullptr && frame.timestamp != last_timestamp)
    {
        last_timestamp = frame.timestamp;
        cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR(frame.height, frame.width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
        cv::Mat left_image, right_image;
        left_image = frameBGR(cv::Rect(0, 0, frame.width / 2, frame.height));
        right_image = frameBGR(cv::Rect(frame.width / 2, 0, frame.width / 2, frame.height));

        cv_bridge::CvImage cv_left_image;
        cv_left_image.image = left_image;
        cv_left_image.encoding = "bgr8";
        cv_left_image.header.frame_id = "left_frame";
        cv_left_image.header.stamp = ros::Time(last_timestamp * 1e-9);
        left_image_pub.publish(cv_left_image.toImageMsg());

        cv_bridge::CvImage cv_right_image;
        cv_right_image.image = right_image;
        cv_right_image.encoding = "bgr8";
        cv_right_image.header.frame_id = "right_frame";
        cv_right_image.header.stamp = ros::Time(last_timestamp * 1e-9);
        right_image_pub.publish(cv_right_image.toImageMsg());
    }
}

void sensor_callback(const ros::TimerEvent &timer_event)
{
    const sl_oc::sensors::data::Imu imuData = zed->getLastIMUData();
    if (imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL) // Uncomment to use only data syncronized with the video frames
    {
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time(imuData.timestamp * 1e-9);
        imu.header.frame_id = "world";
        imu.angular_velocity.x = imuData.gX;
        imu.angular_velocity.y = imuData.gY;
        imu.angular_velocity.z = imuData.gZ;
        imu.linear_acceleration.x = imuData.aX;
        imu.linear_acceleration.y = imuData.aY;
        imu.linear_acceleration.z = imuData.aZ;
        sensor_pub.publish(imu);
    }
}

void correctFramerate(int resolution)
{
    switch (resolution)
    {
    case 0:
        gFps = sl_oc::video::FPS::FPS_15;
        gResolution = sl_oc::video::RESOLUTION::HD2K;
        break;
    case 1:
        gFps = sl_oc::video::FPS::FPS_30;
        gResolution = sl_oc::video::RESOLUTION::HD1080;
        break;
    case 2:
        gFps = sl_oc::video::FPS::FPS_60;
        gResolution = sl_oc::video::RESOLUTION::HD720;
        break;
    case 3:
        gFps = sl_oc::video::FPS::FPS_100;
        gResolution = sl_oc::video::RESOLUTION::VGA;
        break;
    default:
        ROS_FATAL("Unknow resolution passed");
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // setup publisher stuff
    image_transport::ImageTransport it(nh);
    left_image_pub = it.advertise("left/image_raw", 10);
    right_image_pub = it.advertise("right/image_raw", 10);
    sensor_pub = nh.advertise<sensor_msgs::Imu>("imu/raw", 100);

    ros::Timer image_timer;
    ros::Timer sensor_timer;

    // get ros param
    int resolution;
    private_nh.param("resolution", resolution, 1);
    correctFramerate(resolution);
    zed = new StereoCamera(gResolution, gFps);
    if (zed->camera)
    {
        image_timer = nh.createTimer(ros::Duration(0.01), image_callback);
    }
    if (zed->sensor)
    {
        sensor_timer = nh.createTimer(ros::Duration(0.001), sensor_callback);
    }
    if (!zed->camera && !zed->sensor)
    {
        ROS_ERROR("Fail to Initialize the camera");
        return EXIT_FAILURE;
    }
    ros::spin();
}
