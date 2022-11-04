#include "RealSense2IMUDevice.h"
#include <HAL/Utils/TicToc.h>
#include <iostream>

namespace hal
{

RealSense2IMUDevice::RealSense2IMUDevice(rs2::device& device, IMUDriverDataCallback callback) :
  device_(device)
{
  m_ImuCallback = callback;	
  Initialize();
}

RealSense2IMUDevice::~RealSense2IMUDevice()
{
}

/*bool RealSense2IMUDevice::Capture(CameraMsg& images)
{
  frameset_ = pipeline_->wait_for_frames();
  images.set_device_time(frameset_.get_timestamp());
  images.set_system_time(hal::Tic());
  if (capture_ir0_) CaptureInfraredStream(1, images);
  if (capture_ir1_) CaptureInfraredStream(2, images);
  if (capture_color_) CaptureColorStream(images);
  if (capture_depth_) CaptureDepthStream(images);
  return true;
}

std::shared_ptr<CameraDriverInterface> RealSense2IMUDevice::GetInputDevice()
{
  return std::shared_ptr<CameraDriverInterface>();
}

std::string RealSense2IMUDevice::GetDeviceProperty(const std::string&)
{
  return "";
}
*/
size_t RealSense2IMUDevice::NumChannels() const
{
  return streams_.size();
}

/*size_t RealSense2IMUDevice::Width(size_t index) const
{
  return streams_[index].width();
}

size_t RealSense2IMUDevice::Height(size_t index) const
{
  return streams_[index].height();
}

double RealSense2IMUDevice::MaxExposure(int channel) const
{
  const rs2_option option = RS2_OPTION_EXPOSURE;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.max;
}

double RealSense2IMUDevice::MinExposure(int channel) const
{
  const rs2_option option = RS2_OPTION_EXPOSURE;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.min;
}

double RealSense2IMUDevice::MaxGain(int channel) const
{
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.max;
}

double RealSense2IMUDevice::MinGain(int channel) const
{
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  const rs2::option_range range = sensor.get_option_range(option);
  return range.min;
}

double RealSense2IMUDevice::Exposure(int channel)
{
  const rs2_option option = RS2_OPTION_EXPOSURE;
  const rs2::sensor& sensor = GetSensor(channel);
  return sensor.get_option(option);
}

void RealSense2IMUDevice::SetExposure(double exposure, int channel)
{
  (exposure > 0) ?
      DisableAutoExposure(channel, exposure) :
      EnableAutoExposure(channel);
}

double RealSense2IMUDevice::Gain(int channel)
{
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  return sensor.get_option(option);
}

void RealSense2IMUDevice::SetGain(double gain, int channel)
{
  gain_ = gain;
  const rs2_option option = RS2_OPTION_GAIN;
  const rs2::sensor& sensor = GetSensor(channel);
  sensor.set_option(option, gain_);
}

double RealSense2IMUDevice::ProportionalGain(int channel) const
{
  return IsColorStream(channel) ? 0.45 : 2.5;
}

double RealSense2IMUDevice::IntegralGain(int channel) const
{
  return IsColorStream(channel) ? 0.01 : 0.1;
}

double RealSense2IMUDevice::DerivativeGain(int channel) const
{
  return IsColorStream(channel) ? 0.5 : 3.0;
}

double RealSense2IMUDevice::Emitter() const
{
  if (depth_sensor_.get_option(RS2_OPTION_EMITTER_ENABLED))
  {
    const rs2_option option = RS2_OPTION_LASER_POWER;
    const double value = depth_sensor_.get_option(option);
    const rs2::option_range range = depth_sensor_.get_option_range(option);
    return (value - range.min) / (range.max - range.min);
  }

  return 0.0;
}

void RealSense2IMUDevice::SetEmitter(double emitter) const
{
  emitter = std::max(0.0, std::min(1.0, emitter));
  const rs2_option option = RS2_OPTION_LASER_POWER;
  depth_sensor_.set_option(RS2_OPTION_EMITTER_ENABLED, emitter <= 0.0);
  const rs2::option_range range = depth_sensor_.get_option_range(option);
  emitter = range.min + emitter * (range.max - range.min);
  depth_sensor_.set_option(option, emitter);
}

void RealSense2IMUDevice::EnableAutoExposure(int channel)
{
  std::cout << "get sensor:" << std::endl; 
  rs2::sensor& sensor = GetSensor(channel);
  std::cout << "set op auto exp, sensor:" << sensor.get_info(RS2_CAMERA_INFO_NAME) <<std::endl; 
  if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
  	sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, true);
   }
  if (IsColorStream(channel))
  {
	  
    std::cout << "set op auto wb" << std::endl; 
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, true);

    std::cout << "set op auto bc" << std::endl; 
    sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, true);
  }
  std::cout << "enable done" << std::endl;
}

void RealSense2IMUDevice::DisableAutoExposure(int channel, double exposure)
{
  rs2::sensor& sensor = GetSensor(channel);
  sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, false);
  sensor.set_option(RS2_OPTION_EXPOSURE, exposure);

  if (IsColorStream(channel))
  {
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, false);
    sensor.set_option(RS2_OPTION_BACKLIGHT_COMPENSATION, false);
    sensor.set_option(RS2_OPTION_WHITE_BALANCE, 3250);
  }
}

void RealSense2IMUDevice::CaptureInfraredStream(int index, CameraMsg& images)
{
  ImageMsg* image = images.add_image();
  rs2::video_frame frame = frameset_.get_infrared_frame(index);
  const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
  image->set_timestamp(frameset_.get_timestamp());
  image->set_serial_number(serial_number_);
  image->set_width(frame.get_width());
  image->set_height(frame.get_height());
  image->set_data(frame.get_data(), bytes);
  image->set_format(PB_LUMINANCE);
  image->set_type(PB_UNSIGNED_BYTE);
}

void RealSense2IMUDevice::CaptureColorStream(CameraMsg& images)
{
  ImageMsg* image = images.add_image();
  rs2::video_frame frame = frameset_.get_color_frame();
  const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
  image->set_timestamp(frameset_.get_timestamp());
  image->set_serial_number(serial_number_);
  image->set_width(frame.get_width());
  image->set_height(frame.get_height());
  image->set_data(frame.get_data(), bytes);
  image->set_type(PB_UNSIGNED_BYTE);
  image->set_format(PB_RGB);
}

void RealSense2IMUDevice::CaptureDepthStream(CameraMsg& images)
{
  ImageMsg* image = images.add_image();
  rs2::video_frame frame = frameset_.get_depth_frame();
  const size_t bytes = frame.get_stride_in_bytes() * frame.get_height();
  image->set_timestamp(frameset_.get_timestamp());
  image->set_serial_number(serial_number_);
  image->set_width(frame.get_width());
  image->set_height(frame.get_height());
  image->set_data(frame.get_data(), bytes);
  image->set_format(PB_LUMINANCE);
  image->set_type(PB_SHORT);
}

bool RealSense2IMUDevice::IsColorStream(int channel) const
{
  return streams_[channel].stream_type() == RS2_STREAM_COLOR;
}*/

const rs2::sensor& RealSense2IMUDevice::GetSensor(int channel) const
{
  //return IsColorStream(channel) ? color_sensor_ : depth_sensor_;
  return motion_sensor_;
}

rs2::sensor&RealSense2IMUDevice::GetSensor(int channel)
{
  //return IsColorStream(channel) ? color_sensor_ : depth_sensor_;
  return motion_sensor_;
}

void RealSense2IMUDevice::Initialize()
{
  CreateSerialNumber();
  CreatePipeline();
  ConfigurePipeline();
  CreateSensors();
  CreateStreams();
}

void RealSense2IMUDevice::CreateSerialNumber()
{
  serial_string_ = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  serial_number_ = std::strtoull(serial_string_.c_str(), nullptr, 10);
}

void RealSense2IMUDevice::CreatePipeline()
{
  rs2::pipeline* pointer = new rs2::pipeline;
  pipeline_ = std::unique_ptr<rs2::pipeline>(pointer);
}

void RealSense2IMUDevice::ConfigurePipeline()
{
  CreateConfiguration();
  /*if (capture_ir0_) ConfigureInfraredStream(1);
  if (capture_ir1_) ConfigureInfraredStream(2);
  if (capture_color_) ConfigureColorStream();
  if (capture_depth_) ConfigureDepthStream();*/
  ConfigureAccelStream();
  ConfigureGyroStream();
  pipeline_->start(*configuration_, [&](rs2::frame frame)
  {		  
     auto motion = frame.as<rs2::motion_frame>();
     bool valid_accel = false;
     bool valid_gyro = false;
     rs2_vector gyro_data;
     rs2_vector accel_data;
     //std::cout << "IMU callback" << std::endl;
     if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
     {
       gyro_data = motion.get_motion_data();
       valid_gyro = true;
     }
     if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
      { 
        accel_data = motion.get_motion_data();
        valid_accel = true;
      }
		  
     std::cout << "valid:" << valid_accel << "," << valid_gyro << std::endl;	
     if (valid_accel || valid_gyro) 
     {
      	hal::ImuMsg dataIMU;
        {
          //std::unique_lock<std::mutex> lk(dataLock);
          if (valid_accel) {
      	  hal::VectorMsg* pbAccel = dataIMU.mutable_accel();
      	  pbAccel->add_data(accel_data.x);
      	  pbAccel->add_data(accel_data.y);
      	  pbAccel->add_data(accel_data.z);
	  //std::cout << "IMU accel:" << accel_data << std::endl;
	  }

	  if (valid_gyro) {
          hal::VectorMsg* pbGyro = dataIMU.mutable_gyro();
          pbGyro->add_data(gyro_data.x);
          pbGyro->add_data(gyro_data.y);
          pbGyro->add_data(gyro_data.z);
	  //std::cout << "IMU gyro:" << gyro_data << std::endl;
	  }
	  dataIMU.set_device_time(motion.get_timestamp());
          dataIMU.set_system_time(hal::Tic());
       }

       if( m_ImuCallback )
         {
           //std::cout << "IMU:" << dataIMU << std::endl;	   
           m_ImuCallback( dataIMU );
         }
     }	
  });
}

void RealSense2IMUDevice::CreateConfiguration()
{
  rs2::config* pointer = new rs2::config();
  configuration_ = std::unique_ptr<rs2::config>(pointer);
  configuration_->enable_device(serial_string_);
  configuration_->disable_all_streams();
}


void RealSense2IMUDevice::ConfigureAccelStream()
{
  configuration_->enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F,63);
}


void RealSense2IMUDevice::ConfigureGyroStream()
{
  configuration_->enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F,63);
}

/*void RealSense2IMUDevice::ConfigureInfraredStream(int index)
{
  const int rate = frame_rate_;
  const rs2_format format = RS2_FORMAT_Y8;
  const rs2_stream stream = RS2_STREAM_INFRARED;
  configuration_->enable_stream(stream, index, width_, height_, format, rate);
}

void RealSense2IMUDevice::ConfigureColorStream()
{
  const int rate = frame_rate_;
  const rs2_format format = RS2_FORMAT_RGB8;
  const rs2_stream stream = RS2_STREAM_COLOR;
  configuration_->enable_stream(stream, width_, height_, format, rate);
}

void RealSense2IMUDevice::ConfigureDepthStream()
{
  const int rate = frame_rate_;
  const rs2_format format = RS2_FORMAT_Z16;
  const rs2_stream stream = RS2_STREAM_DEPTH;
  configuration_->enable_stream(stream, width_, height_, format, rate);
}*/

void RealSense2IMUDevice::CreateSensors()
{
  std::vector<rs2::sensor> sensors = device_.query_sensors();

  for (rs2::sensor sensor: sensors)
  {
    if (sensor.is<rs2::motion_sensor>()) 
        motion_sensor_ = sensor;
  }
}

void RealSense2IMUDevice::CreateStreams()
{
  /*if (capture_ir0_) CreateInfraredStream(1);
  if (capture_ir1_) CreateInfraredStream(2);
  if (capture_color_) CreateColorStream();
  if (capture_depth_) CreateDepthStream();*/

  rs2::pipeline_profile profile = pipeline_->get_active_profile();
  rs2::stream_profile accel_stream = profile.get_stream(RS2_STREAM_ACCEL);
  streams_.push_back(accel_stream.as<rs2::motion_stream_profile>());
  
  rs2::stream_profile gyro_stream = profile.get_stream(RS2_STREAM_GYRO);
  streams_.push_back(gyro_stream.as<rs2::motion_stream_profile>());
}

/*void RealSense2IMUDevice::CreateInfraredStream(int index)
{
  rs2::pipeline_profile profile = pipeline_->get_active_profile();
  rs2::stream_profile stream = profile.get_stream(RS2_STREAM_INFRARED, index);
  streams_.push_back(stream.as<rs2::video_stream_profile>());
}

void RealSense2IMUDevice::CreateColorStream()
{
  rs2::pipeline_profile profile = pipeline_->get_active_profile();
  rs2::stream_profile stream = profile.get_stream(RS2_STREAM_COLOR);
  streams_.push_back(stream.as<rs2::video_stream_profile>());
}

void RealSense2IMUDevice::CreateDepthStream()
{
  rs2::pipeline_profile profile = pipeline_->get_active_profile();
  rs2::stream_profile stream = profile.get_stream(RS2_STREAM_DEPTH);
  streams_.push_back(stream.as<rs2::video_stream_profile>());
}*/

} // namespace hal
