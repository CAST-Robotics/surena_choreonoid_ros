#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <sensor_msgs/Image.h>
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
    ros::Publisher rangeImagePublisher_;
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
        rangeImagePublisher_ = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);

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
        vision.header.stamp.fromSec(io_->currentTime());
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
    range.header.stamp.fromSec(io_->currentTime());
    range.header.frame_id = sensor->name();
    range.width = sensor->resolutionX();
    range.height = sensor->resolutionY();
    range.is_bigendian = false;
    range.is_dense = true;
    range.row_step = range.point_step * range.width;
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
}

    virtual bool control() override
    {
        timeCounter += timeStep;
        if(timeCounter >= 0.05){
            for(size_t i=0; i < cameras.size(); ++i){
                Camera* camera = cameras[i];
                std::string filename = camera->name() + ".png";
                updateVisionSensor(camera);
                camera->constImage().save(filename);
                *os << "The image of " << camera->name()
                    << " has been saved to \"" << filename << "\"." << std::endl;
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
