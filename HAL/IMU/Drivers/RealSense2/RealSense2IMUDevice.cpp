#include "RealSense2IMUDevice.h"
#include <HAL/Utils/TicToc.h>
#include <iostream>
#include <deque>

namespace hal
{

RealSense2IMUDevice::RealSense2IMUDevice(rs2::device& device) :
  device_(device)
{
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


void RealSense2IMUDevice::RegisterIMUDataCallback(IMUDriverDataCallback callback) 
{
   m_ImuCallback = callback;	 
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
     if (!motion) {
        std::cout << "Not a motion frame is NULL" << std::endl;
    	return;
     }
     
     auto stream = motion.get_profile().stream_type();
     if (stream != RS2_STREAM_GYRO && stream != RS2_STREAM_ACCEL) {
        std::cout << "Stream is neither GYRO or ACCEL" << std::endl;
        return;
     }
     auto stream_index = (stream == GYRO.first) ? GYRO : ACCEL;
     auto format = motion.get_profile().format();
     if (format != RS2_FORMAT_MOTION_XYZ32F) {
        std::cout << "Format is not XYZ32F" << std::endl;
        return;
     }
     
     double frame_time = motion.get_timestamp();
	
     auto data = motion.get_motion_data();
     Eigen::Vector3d v(data.x, data.y, data.z);

     auto timestamp = motion.get_timestamp();
     auto timestamp_domain = motion.get_frame_timestamp_domain();
        
     if (timestamp_domain != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
	std::cout << "timestamp domain is not Global time " << std::endl;      
     	return;	     
     }

     std::deque<hal::ImuMsg> imu_msgs;
     CimuData imu_data(stream_index, v, timestamp*1e6);
     FillImuData_LinearInterpolation(imu_data, imu_msgs);

     while (imu_msgs.size())
       {	
       	if( m_ImuCallback )
         {
           hal::ImuMsg dataIMU = imu_msgs.front();
           m_ImuCallback( dataIMU );
         }
	 imu_msgs.pop_front();
       }	
  });
}

template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

void RealSense2IMUDevice::FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<hal::ImuMsg>& imu_msgs)
{
    static std::deque<CimuData> _imu_history;
    _imu_history.push_back(imu_data);
    stream_index_pair type(imu_data.m_type);
    imu_msgs.clear();

    if ((type != ACCEL) || _imu_history.size() < 3)
        return;

    std::deque<CimuData> gyros_data;
    CimuData accel0, accel1, crnt_imu;

    while (_imu_history.size())
    {
        crnt_imu = _imu_history.front();
        _imu_history.pop_front();
        if (!accel0.is_set() && crnt_imu.m_type == ACCEL)
        {
            accel0 = crnt_imu;
        }
        else if (accel0.is_set() && crnt_imu.m_type == ACCEL)
        {
            accel1 = crnt_imu;
            const double dt = accel1.m_time_ns - accel0.m_time_ns;

            while (gyros_data.size())
            {
                CimuData crnt_gyro = gyros_data.front();
                gyros_data.pop_front();
                const double alpha = (crnt_gyro.m_time_ns - accel0.m_time_ns) / dt;
                CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time_ns);
                imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
            }
            accel0 = accel1;
        }
        else if (accel0.is_set() && crnt_imu.m_time_ns >= accel0.m_time_ns && crnt_imu.m_type == GYRO)
        {
            gyros_data.push_back(crnt_imu);
        }
    }
    _imu_history.push_back(crnt_imu);
    return;
}


hal::ImuMsg RealSense2IMUDevice::CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
   hal::ImuMsg dataIMU;
   hal::VectorMsg* pbAccel = dataIMU.mutable_accel();
   pbAccel->add_data(accel_data.m_data.x());
   pbAccel->add_data(accel_data.m_data.y());
   pbAccel->add_data(accel_data.m_data.z());

   hal::VectorMsg* pbGyro = dataIMU.mutable_gyro();
   pbGyro->add_data(gyro_data.m_data.x());
   pbGyro->add_data(gyro_data.m_data.y());
   pbGyro->add_data(gyro_data.m_data.z());
	  
   dataIMU.set_device_time(gyro_data.m_time_ns*1e-6);
   dataIMU.set_system_time(hal::Tic());
   return dataIMU;
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
  configuration_->enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
}


void RealSense2IMUDevice::ConfigureGyroStream()
{
  configuration_->enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
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
