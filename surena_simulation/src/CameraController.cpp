#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

using namespace cnoid;

class CameraController : public SimpleController
{
    DeviceList<Camera> cameras;
    DeviceList<RangeCamera> rangeCameras;
    double timeCounter;
    double timeStep;
    std::ostream* os;
    SimpleControllerIO* io_;
    image_transport::Publisher imagePublisher_;
    image_transport::Publisher depthImagePublisher_;
    ros::Publisher rangeImagePublisher_;
    ros::Publisher cameraInfoPublisher_;
    ros::NodeHandle nh;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        os = &io->os();
        io_ = io;
        cameras << io->body()->devices();
        rangeCameras << io->body()->devices();
        image_transport::ImageTransport it(nh);
        imagePublisher_ = it.advertise("/image_raw", 1);
        depthImagePublisher_ = it.advertise("/depth_image_raw", 1);
        rangeImagePublisher_ = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
        cameraInfoPublisher_ = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);

        for(size_t i=0; i < cameras.size(); ++i){
            Device* camera = cameras[i];
            io->enableInput(camera);
            *os << "Device type: " << camera->typeName()
                << ", id: " << camera->id()
                << ", name: " << camera->name() << std::endl;
        }

        for(size_t i=0; i < rangeCameras.size(); ++i){
            Device* range_camera = rangeCameras[i];
            io->enableInput(range_camera);
            *os << "Device type: " << range_camera->typeName()
                << ", id: " << range_camera->id()
                << ", name: " << range_camera->name() << std::endl;
        }

        timeCounter = 0.0;
        timeStep = io->timeStep();

        return true;
    }

    void updateVisionSensor(Camera* sensor)
    {
        sensor_msgs::Image vision;
        vision.header.stamp = ros::Time::now();
        vision.header.frame_id = sensor->name();
        vision.height = sensor->image().height();
        vision.width = sensor->image().width();
        
        if (sensor->image().numComponents() == 3)
            vision.encoding = sensor_msgs::image_encodings::RGB8;
        else if (sensor->image().numComponents() == 1)
            vision.encoding = sensor_msgs::image_encodings::MONO8;
        else {
            ROS_WARN("unsupported image component number: %i", sensor->image().numComponents());
        }
        
        vision.is_bigendian = 0;
        vision.step = sensor->image().width() * sensor->image().numComponents();
        
        vision.data.resize(vision.step * vision.height);
        std::memcpy(&(vision.data[0]), &(sensor->image().pixels()[0]), vision.step * vision.height);
        
        imagePublisher_.publish(vision);
    }

    void updateRangeVisionSensor(RangeCamera* sensor)
    {
        sensor_msgs::PointCloud2 range;
        sensor_msgs::Image depth;
        depth.header.stamp = ros::Time::now();
        range.header.stamp = ros::Time::now();
        range.header.frame_id = sensor->name();
        depth.header.frame_id = sensor->name();

        depth.width = sensor->resolutionX();
        depth.height = sensor->resolutionY();
        depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depth.is_bigendian = false;
        const int point_step = 4;
        depth.step = point_step * depth.width;
        depth.data.resize(depth.step * depth.height);

        const std::vector<Vector3f>& points1 = sensor->constPoints();
        unsigned char* dst1 = (unsigned char*)&(depth.data[0]);
        for (size_t j = 0; j < points1.size(); ++j) {
            float z = - points1[j].z();
            std::memcpy(&dst1[0], &z, point_step);
            dst1 += point_step;
        }

        range.width = sensor->resolutionX();
        range.height = sensor->resolutionY();
        range.is_bigendian = false;
        range.is_dense = true;
        if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
            range.fields.resize(6);
            range.fields[3].name = "rgb";
            range.fields[3].offset = 12;
            range.fields[3].count = 1;
            range.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            range.point_step = 16;
        } else {
            range.fields.resize(3);
            range.point_step = 12;
        }
        range.row_step = range.point_step * range.width;
        range.fields[0].name = "x";
        range.fields[0].offset = 0;
        range.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        range.fields[0].count = 4;
        range.fields[1].name = "y";
        range.fields[1].offset = 4;
        range.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        range.fields[1].count = 4;
        range.fields[2].name = "z";
        range.fields[2].offset = 8;
        range.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        range.fields[2].count = 4;
        const std::vector<Vector3f>& points = sensor->constPoints();
        const unsigned char* pixels = sensor->constImage().pixels();
        range.data.resize(points.size() * range.point_step);
        unsigned char* dst = (unsigned char*)&(range.data[0]);
        for (size_t j = 0; j < points.size(); ++j) {
            float x = points[j].x();
            float y = - points[j].y();
            float z = - points[j].z();
            std::memcpy(&dst[0], &x, 4);
            std::memcpy(&dst[4], &y, 4);
            std::memcpy(&dst[8], &z, 4);
            if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
                dst[14] = *pixels++;
                dst[13] = *pixels++;
                dst[12] = *pixels++;
                dst[15] = 0;
            }
            dst += range.point_step;
        }
        rangeImagePublisher_.publish(range);
        depthImagePublisher_.publish(depth);
    }

    void publishCameraInfo()
    {
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.stamp = ros::Time::now();

        camera_info.distortion_model = "plumb_bob";
        camera_info.height = 480;
        camera_info.width = 640;

        camera_info.K = {554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0};
        camera_info.D = {1e-08, 1e-08, 1e-08, 1e-08, 1e-08};
        camera_info.P = {554.254691191187, 0.0, 320.5, -0.0, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
        camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info.binning_x = 0;
        camera_info.binning_y = 0;
        camera_info.roi.do_rectify = false;
        camera_info.roi.height = 0;
        camera_info.roi.width = 0;
        camera_info.roi.x_offset = 0;
        camera_info.roi.y_offset = 0;
        
        cameraInfoPublisher_.publish(camera_info);
    }

    virtual bool control() override
    {
        timeCounter += timeStep;
        if(timeCounter >= 1.0 / 30.0){
            for(size_t i=0; i < cameras.size(); ++i){
                Camera* camera = cameras[i];
                updateVisionSensor(camera);
                publishCameraInfo();
            }

            for(size_t i=0; i < rangeCameras.size(); ++i){
                RangeCamera* range_camera = rangeCameras[i];
                updateRangeVisionSensor(range_camera);
            }

            timeCounter = 0.0;
        }
        return false;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)
