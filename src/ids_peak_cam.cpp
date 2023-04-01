#include"ids_peak_cam.h"
#include <boost/format.hpp>

ids_cam::ids_cam(ros::NodeHandle nh):
  nh_(nh)
  //cinfo_(new camera_info_manager::CameraInfoManager(nh_)),
  //it_(new image_transport::ImageTransport(nh_)),
  //img_pub_(it_->advertiseCamera("ids_cam/image_raw",1))
{
   cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_));
   it_.reset(new image_transport::ImageTransport(nh_));
   img_pub_=it_->advertiseCamera("image_raw",1);
   
   nh_.param<int>("/frameRate", frameRate_,20);
   nh_.param<int>("/exposureTime", exposureTime_,20000);
   nh_.param<std::string>("/Serial_NO", serNo_,"4104469183");

   camOpen();
   camConfig();

}

ids_cam::~ids_cam()
{
  camAcquisitionStop();
  camClose();
}


void ids_cam::camOpen()
{
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
    if (descriptor->SerialNumber() == serNo_)
    {
        dev_ = descriptor->OpenDevice(peak::core::DeviceAccessType::Control);
        nodeMap_ = dev_->RemoteDevice()->NodeMaps().at(0);

        break;
    }
  }

  if(dev_ == nullptr)
  {
	  ROS_ERROR("SerNO Wrong. Exiting node.");
     peak::Library::Close();
     ros::shutdown();
  }


  try
  {
            // Open standard data stream
    dStream_ = dev_->DataStreams().at(0)->OpenDataStream();
  }
  catch (const std::exception& e)
  {
            // Open data stream failed
     dev_.reset();
     std::cout << "Failed to open DataStream: " << e.what() << std::endl;
     std::cout << "Exiting program." << std::endl << std::endl;

     peak::Library::Close();
     //return 0;
  }

  //ROS_INFO("%s Opened.", serNo_);
  //std::cout<<serNo<<" opened."<<std::endl;
  //
  if(!cinfo_->setCameraName(serNo_))
  {
    ROS_WARN_STREAM("["<<serNo_<<"] name not valid"
        <<" for camera_info_manager");
  }
}

void ids_cam::camConfig()
{

  try
  {
      nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector")
          ->SetCurrentEntry("Default");
      nodeMap_->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->Execute();
	    //load default set

      // wait until the UserSetLoad command has been finished
      nodeMap_->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->WaitUntilDone();

  }
  catch (const std::exception&)
  {
      // UserSet is not available, try to disable ExposureStart or FrameStart trigger manually
      std::cout << "Failed to load UserSet Default. Manual freerun configuration." << std::endl;

      try
      {
          nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
              ->SetCurrentEntry("ExposureStart");
          nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
              ->SetCurrentEntry("Off");
      }
      catch (const std::exception&)
      {
          try
          {
              nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
                  ->SetCurrentEntry("FrameStart");
              nodeMap_->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
                  ->SetCurrentEntry("Off");
          }
          catch (const std::exception&)
          {
              // There is no known trigger available, continue anyway.
          }
      }
  }


        // allocate and announce image buffers
  auto payloadSize = nodeMap_->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
  auto bufferCountMax = dStream_->NumBuffersAnnouncedMinRequired();
  for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
  {
      auto buffer = dStream_->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
      dStream_->QueueBuffer(buffer);
  }
   
  // set a frame rate to 10fps (or max value) since some of the trigger cases require a defined frame rate
  auto frameRateMax = nodeMap_->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
         ->Maximum();

  nodeMap_->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
     ->SetValue(std::min(static_cast<double>(frameRate_), frameRateMax));

  auto exposureTimeMax = nodeMap_->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
    ->Maximum();

  nodeMap_->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
     ->SetValue(std::min(static_cast<double>(exposureTime_), exposureTimeMax));


  // Lock critical features to prevent them from changing during acquisition
  nodeMap_->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

  // start acquisition
  dStream_->StartAcquisition(peak::core::AcquisitionStartMode::Default, peak::core::DataStream::INFINITE_NUMBER);
  nodeMap_->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();

	double exposureTime_current = nodeMap_->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Value();
	double frameRate_current = nodeMap_->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value();

  ROS_INFO("CURRENT EXPOSURE TIME: %f", exposureTime_current);
  ROS_INFO("CURRENT EXPOSURE TIME: %f", frameRate_current);
}

void ids_cam::camAcquisition(sensor_msgs::Image &image)
{
   auto buffer = dStream_->WaitForFinishedBuffer(5000);
   auto dev_image = peak::BufferTo<peak::ipl::Image>(buffer);

	 //show the image by opencv 
   //imgae_ = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);
   //
   //
   image.height = dev_image.Height();
   image.width = dev_image.Width();
   image.step = dev_image.Width();
   
   image.encoding = sensor_msgs::image_encodings::MONO8;
   int image_size =dev_image.Height()*dev_image.Width(); 

   int sizeBuffer = static_cast<int>(dev_image.ByteCount());
   image.data.resize(dev_image.Height()*dev_image.Width());

   // Device buffer is being copied into cv::Mat array
   std::memcpy(&image.data[0], dev_image.Data(), static_cast<size_t>(sizeBuffer));

   // queue buffer
   dStream_->QueueBuffer(buffer);
}


void ids_cam::camAcquisitionStop()
{

  // stop acquistion of camera
  try
  {
      dStream_->StopAcquisition(peak::core::AcquisitionStopMode::Default);
  }
  catch (const std::exception&)
  {
      // Some transport layers need no explicit acquisition stop of the datastream when starting its
      // acquisition with a finite number of images. Ignoring Errors due to that TL behavior.

  ROS_WARN("WARNING: Ignoring that TL failed to stop acquisition on datastream.");
  }
  nodeMap_->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();

  // Unlock parameters after acquisition stop
  nodeMap_->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(0);

  // flush and revoke all buffers
  dStream_->Flush(peak::core::DataStreamFlushMode::DiscardAll);
  for (const auto& buffer : dStream_->AnnouncedBuffers())
  {
      dStream_->RevokeBuffer(buffer);
  }
}
    
void ids_cam::camClose()
{
  peak::Library::Close();
}

void ids_cam::publish(const sensor_msgs::ImagePtr &image)
{
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

  img_pub_.publish(image, ci);

}

void ids_cam::poll()
{
  sensor_msgs::ImagePtr image(new sensor_msgs::Image);
  camAcquisition(*image);
  publish(image);
}

void ids_cam::shutdown()
{
  camAcquisitionStop();
  camClose();
}
