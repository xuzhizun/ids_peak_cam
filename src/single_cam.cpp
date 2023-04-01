
#define VERSION "1.2.0"

#include <cstdint>
#include <iostream>
#include <cstdlib>

#include <peak/converters/peak_buffer_converter_ipl.hpp>
#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <csignal>


std::shared_ptr<peak::core::Device> device = nullptr;
std::shared_ptr<peak::core::NodeMap> nodeMapRemoteDevice = nullptr;
std::shared_ptr<peak::core::DataStream> dataStream=nullptr;

bool continuousSample = true;

void initializeCam(std::string serNO);
void camConfig(const int frameRate_, const int exposureTime_);
cv::Mat camAcquisition();
void camAcquisitionStop();
void camClose();
void keywait();
void signal_callback(int signum);

int main(int argc, char** argv)
{

  ros::init(argc, argv, "single_cam");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  int frameRate, exposureTime;
  std::string serNO;

  nh.param<int>("/frameRate", frameRate, 60);
  nh.param<int>("/exposureTime", exposureTime, 2000);
  nh.param<std::string>("/Serial_NO", serNO, "4104469183");

  image_transport::Publisher pub = it.advertise("ids_cam/image_raw", 1);

  if(!ros::isShuttingDown())
  {

    initializeCam(serNO);
    std::cout << "Camera Initialized" << std::endl;
    camConfig(frameRate, exposureTime);
    std::cout << "Camera Configured" << std::endl;

    ROS_INFO("Press enter to exit.");
    std::thread interrupt(keywait);
    interrupt.detach();
  }

  while (nh.ok()) 
  {
     cv::Mat imageCV = camAcquisition(); 
    // Check if grabbed frame is actually full with some content
     if(!imageCV.empty()) 
     {
      auto msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageCV).toImageMsg();
      pub.publish(msg);
     }

    if(!continuousSample)
      break;
  }

  camAcquisitionStop();

  camClose();

  return 0;
}


void initializeCam(std::string serNo)
{
  
  std::cout << "IDS peak API \" single cam ros node \" Sample v" << VERSION << std::endl;

  peak::Library::Initialize();
  auto& deviceManager = peak::DeviceManager::Instance();
  deviceManager.Update();
 
  if (deviceManager.Devices().empty())
  {
      ROS_ERROR("No camera found. Exiting program.");
      peak::Library::Close();
      //std::exit(0);
      ros::shutdown();
      //return 0;
  }

  for (const auto& descriptor: deviceManager.Devices())
  {
    if (descriptor->SerialNumber() == serNo)
    {
        device = descriptor->OpenDevice(peak::core::DeviceAccessType::Control);
        nodeMapRemoteDevice = device->RemoteDevice()->NodeMaps().at(0);

        break;
    }
  }

  if(device == nullptr)
  {
	  ROS_ERROR("SerNO Wrong. Exiting node.");
     peak::Library::Close();
     //std::exit(0);
     ros::shutdown();
  }


  try
  {
            // Open standard data stream
    dataStream = device->DataStreams().at(0)->OpenDataStream();
  }
  catch (const std::exception& e)
  {
            // Open data stream failed
     device.reset();
     std::cout << "Failed to open DataStream: " << e.what() << std::endl;
     std::cout << "Exiting program." << std::endl << std::endl;

     peak::Library::Close();
     //return 0;
  }

  std::cout<<serNo<<" Connected"<<std::endl;

}

void camConfig(const int frameRate_, const int exposureTime_)
{
        try
        {
            nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector")
                ->SetCurrentEntry("Default");
            nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->Execute();
	    //load default set

            // wait until the UserSetLoad command has been finished
            nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->WaitUntilDone();

        }
        catch (const std::exception&)
        {
            // UserSet is not available, try to disable ExposureStart or FrameStart trigger manually
            std::cout << "Failed to load UserSet Default. Manual freerun configuration." << std::endl;

            try
            {
                nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
                    ->SetCurrentEntry("ExposureStart");
                nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
                    ->SetCurrentEntry("Off");
            }
            catch (const std::exception&)
            {
                try
                {
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
                        ->SetCurrentEntry("FrameStart");
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
                        ->SetCurrentEntry("Off");
                }
                catch (const std::exception&)
                {
                    // There is no known trigger available, continue anyway.
                }
            }
        }


        // allocate and announce image buffers
        auto payloadSize = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
        auto bufferCountMax = dataStream->NumBuffersAnnouncedMinRequired();
        for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
        {
            auto buffer = dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
            dataStream->QueueBuffer(buffer);
        }

        // set a frame rate to 10fps (or max value) since some of the trigger cases require a defined frame rate
        //auto frameRateMax = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
                                //->Maximum();
        //nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
        //   ->SetValue(std::min(10.0, frameRateMax));

	//set frame rate to max value
       // nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
        //   ->SetValue(frameRateMax);

        nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(exposureTime_);
	    //set the exposure time

        nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(frameRate_);
	    //set the framerate
        // define the number of images to acquire

        // Lock critical features to prevent them from changing during acquisition
        nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

        // start acquisition
        dataStream->StartAcquisition(peak::core::AcquisitionStartMode::Default, peak::core::DataStream::INFINITE_NUMBER);
        nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();

	double exposureTime_current = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Value();
	double frameRate_current = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value();

	std::cout<<"Current Exposure Time: "<<exposureTime_current<<'\n'
		<<"Current FrameRate : "<<frameRate_current<<'\n';
        // process the acquired images
}

cv::Mat camAcquisition()
{
            auto buffer = dataStream->WaitForFinishedBuffer(5000);
            auto image = peak::BufferTo<peak::ipl::Image>(buffer);

	    //show the image by opencv 
	    //
	    cv::Mat cvImage;
            cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);

            int sizeBuffer = static_cast<int>(image.ByteCount());
            // Device buffer is being copied into cv::Mat array
            std::memcpy(cvImage.data, image.Data(), static_cast<size_t>(sizeBuffer));

    // queue buffer
    dataStream->QueueBuffer(buffer);
    return cvImage;
}


void camAcquisitionStop()
{

        // stop acquistion of camera
        try
        {
            dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
        }
        catch (const std::exception&)
        {
            // Some transport layers need no explicit acquisition stop of the datastream when starting its
            // acquisition with a finite number of images. Ignoring Errors due to that TL behavior.

         ROS_ERROR("WARNING: Ignoring that TL failed to stop acquisition on datastream.");
        }
        nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();

        // Unlock parameters after acquisition stop
        nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(0);

        // flush and revoke all buffers
        dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
        for (const auto& buffer : dataStream->AnnouncedBuffers())
        {
            dataStream->RevokeBuffer(buffer);
        }
}

void camClose()
{
    peak::Library::Close();
}

void keywait()
{
    system("read _");
    continuousSample = false;
}

