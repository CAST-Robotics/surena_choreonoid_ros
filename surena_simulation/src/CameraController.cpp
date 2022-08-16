#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

using namespace cnoid;

class CameraController : public SimpleController
{
    DeviceList<Camera> cameras;
    double timeCounter;
    double timeStep;
    std::ostream* os;
    SimpleControllerIO* io_;
    image_transport::Publisher imagePublisher_;
    ros::NodeHandle nh;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        os = &io->os();
        io_ = io;
        cameras << io->body()->devices();
        image_transport::ImageTransport it(nh);
        imagePublisher_ = it.advertise("/image_raw", 1);
        for(size_t i=0; i < cameras.size(); ++i){
            Device* camera = cameras[i];
            io->enableInput(camera);
            *os << "Device type: " << camera->typeName()
                << ", id: " << camera->id()
                << ", name: " << camera->name() << std::endl;
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
            timeCounter = 0.0;
        }
        return false;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)
