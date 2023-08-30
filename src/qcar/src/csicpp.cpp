// header files for configuring CSI node
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "quanser_types.h"
#include "quanser_memory.h"
#include "quanser_messages.h"
#include "quanser_signal.h"
#include "quanser_timer.h"
#include "quanser_video.h"


int main(int argc, char **argv) //int main(int argc, char * argv[]) <-- this is what showned in quanser doc
{
    // configure camera settings
    const t_uint32 frame_width  = 820;
    const t_uint32 frame_height = 410;
    const t_uint32 frame_rate	= 120;
    t_video_capture_attribute video_attribute;
    t_error result;
    t_video_capture capture;

    //setup publisher information
    ros::init(argc, argv, "csi_cpp_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher csi_front_pub = it.advertise("/qcar/csi_image", 1);
    ros::Rate loop_rate(frame_rate);

    //memory allocation for csi image
    t_uint8 *csi_front;
    csi_front = (t_uint8 *) memory_allocate(frame_width * frame_height * 3 * sizeof(t_uint8));

    // open video camera instace with the preconfigured settings, currently set to camera 0
    // to change the csi camera change cameraURI to:
    // "video://localhost:0" -> right camera
    // "video://localhost:1" -> back camera
    // "video://localhost:2" -> left camera
    // "video://localhost:3" -> front camera

    char *cameraURI = "video://localhost:0";

    // Open video capture device with defined camera parameters
    result = video_capture_open(cameraURI, frame_rate,
                                frame_width, frame_height, IMAGE_FORMAT_ROW_MAJOR_INTERLEAVED_BGR,
                                IMAGE_DATA_TYPE_UINT8, &capture, &video_attribute,0);

    if (result >= 0)
    {
        result = video_capture_start(capture);

        sensor_msgs::ImagePtr img_msg;
        std_msgs::Header header;
        while (ros::ok())
        {
            // reading camera data
            result = video_capture_read(capture,csi_front);

            // format ros image topic data
            cv::Size size(frame_width, frame_height);
            cv::Mat cv_csi_front(size,CV_8UC3, csi_front);

            // publish image data when a frame is available
            if(!cv_csi_front.empty()){
                header.stamp = ros::Time::now();
                img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_csi_front) .toImageMsg();
                csi_front_pub.publish(img_msg);
            }
            else{
                // Close camera info if no image is present
                video_capture_stop(capture);
                video_capture_close(capture);
            }
            cv::waitKey(100);
            ros::spinOnce();
            loop_rate.sleep();
        }


    }
    else {
        std::cout<<"Issue setting up CSI camera"<< std::endl;
    }

    // close camera when camera is not available, release image memory
    video_capture_stop(capture);
    video_capture_close(capture);
    memory_free(csi_front);
}
