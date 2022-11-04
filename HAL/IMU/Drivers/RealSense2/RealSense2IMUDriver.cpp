#include "RealSense2IMUDriver.h"
#include "RealSense2IMUDevice.h"

#include <iostream>
#include <stdexcept>
namespace hal
{

RealSense2IMUDriver::RealSense2IMUDriver(const std::vector<std::string>& ids) :
  ids_(ids)
{
  if (ids_.empty())
  	throw std::runtime_error("Must provide id0=XXXX serial number");	  
	
  if (ids_.size() > 1)
  	throw std::runtime_error("IMU hal only support 1 device at the moment");	  

	
  Initialize();
}

RealSense2IMUDriver::~RealSense2IMUDriver()
{
}


void RealSense2IMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback) {
  m_ImuCallback = callback;
}


/*bool RealSense2IMUDriver::Capture(CameraMsg& images)
{
  for (size_t i = 0; i < devices_.size(); ++i)
  {
    if (!devices_[i]->Capture(images)) return false;
  }

  return true;
}

std::shared_ptr<CameraDriverInterface> RealSense2IMUDriver::GetInputDevice()
{
  return nullptr;
}

std::string RealSense2IMUDriver::GetDeviceProperty(const std::string&)
{
  return "";
}

size_t RealSense2IMUDriver::NumChannels() const
{
  return channel_count_;
}

size_t RealSense2IMUDriver::Width(size_t index) const
{
  return Device(index)->Width(channel_map_[index]);
}

size_t RealSense2IMUDriver::Height(size_t index) const
{
  return Device(index)->Height(channel_map_[index]);
}

double RealSense2IMUDriver::MaxExposure(int channel) const
{
  return Device(channel)->MaxExposure(channel_map_[channel]);
}

double RealSense2IMUDriver::MinExposure(int channel) const
{
  return Device(channel)->MinExposure(channel_map_[channel]);
}

double RealSense2IMUDriver::MaxGain(int channel) const
{
  return Device(channel)->MaxGain(channel_map_[channel]);
}

double RealSense2IMUDriver::MinGain(int channel) const
{
  return Device(channel)->MinGain(channel_map_[channel]);
}

double RealSense2IMUDriver::Exposure(int channel)
{
  return Device(channel)->Exposure(channel_map_[channel]);
}

void RealSense2IMUDriver::SetExposure(double exposure, int channel)
{
  Device(channel)->SetExposure(exposure, channel_map_[channel]);
}

double RealSense2IMUDriver::Gain(int channel)
{
  return Device(channel)->Gain(channel_map_[channel]);
}

void RealSense2IMUDriver::SetGain(double gain, int channel)
{
  Device(channel)->SetGain(gain, channel_map_[channel]);
}

double RealSense2IMUDriver::ProportionalGain(int channel) const
{
  return Device(channel)->ProportionalGain(channel_map_[channel]);
}

double RealSense2IMUDriver::IntegralGain(int channel) const
{
  return Device(channel)->IntegralGain(channel_map_[channel]);
}

double RealSense2IMUDriver::DerivativeGain(int channel) const
{
  return Device(channel)->DerivativeGain(channel_map_[channel]);
}

double RealSense2IMUDriver::Emitter(int device) const
{
  return devices_[device]->Emitter();
}

void RealSense2IMUDriver::SetEmitter(int device, double emitter) const
{
  devices_[device]->SetEmitter(emitter);
}*/

size_t RealSense2IMUDriver::NumDevices() const
{
  return devices_.size();
} 

std::shared_ptr<RealSense2IMUDevice> RealSense2IMUDriver::Device(int channel)
{
  return devices_[device_map_[channel]];
}

std::shared_ptr<const RealSense2IMUDevice> RealSense2IMUDriver::Device(
    int channel) const
{
  return devices_[device_map_[channel]];
}

bool RealSense2IMUDriver::ValidDevice(rs2::device& device, const std::string& id)
{
  if (!ValidDevice(device)) return false;
  return (id == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
}

bool RealSense2IMUDriver::ValidDevice(rs2::device& device)
{
  static const std::string prefix = "Intel RealSense";
  if (!device.supports(RS2_CAMERA_INFO_NAME)) return false;
  const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
  auto result = std::mismatch(prefix.begin(), prefix.end(), name.begin());
  return result.first == prefix.end();
}

void RealSense2IMUDriver::Initialize()
{
  std::cout << "CreateDevice" << std::endl;		
  CreateDevices();
  std::cout << "Set ChannelCount" << std::endl;
  SetChannelCount();
  std::cout << "Create Mapping" << std::endl;
  CreateMapping();
}

void RealSense2IMUDriver::CreateDevices()
{
 
  (ids_.empty()) ? CreateAllDevices() : CreateSelectedDevices();
}

void RealSense2IMUDriver::CreateSelectedDevices()
{
  rs2::context context;
  rs2::device_list devices = context.query_devices();

   for (const std::string& id : ids_)
  {
    for (rs2::device&& device : devices)
    {
     if (ValidDevice(device, id))
      {
        devices_.push_back(std::make_shared<RealSense2IMUDevice>(device, m_ImuCallback));
      }
    }
  }
}

void RealSense2IMUDriver::CreateAllDevices()
{
  rs2::context context;
  rs2::device_list devices = context.query_devices();

  for (rs2::device&& device : devices)
  {
    if (ValidDevice(device))
    {
      devices_.push_back(std::make_shared<RealSense2IMUDevice>(device, m_ImuCallback));
    }
  }
}

void RealSense2IMUDriver::SetChannelCount()
{
  std::cout << "guoqing: SetChannelCount:" << devices_.size() << std::endl;
  channel_count_ = 0;

  for (size_t i = 0; i < devices_.size(); ++i)
  {
    channel_count_ += devices_[i]->NumChannels();
  }
  std::cout << "guoqing: channel_count:" << channel_count_ << std::endl; 
}

void RealSense2IMUDriver::CreateMapping()
{
  size_t index = 0;
  device_map_.resize(channel_count_);
  channel_map_.resize(channel_count_);
 
  std::cout <<  "guoqing: channel_count_" << channel_count_ << std::endl;
  for (size_t i = 0; i < devices_.size(); ++i)
  {
    const size_t count = devices_[i]->NumChannels();
    std::cout << "guoqing: num channels" << count << std::endl; 

    for (size_t j = 0; j < count; ++j)
    {
      device_map_[index] = i;
      channel_map_[index] = j;
      ++index;
    }
  }
}

} // namespace hal
