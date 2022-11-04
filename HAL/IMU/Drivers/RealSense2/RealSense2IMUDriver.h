#pragma once

#include <librealsense2/rs.hpp>
#include <HAL/IMU/IMUDriverInterface.h>

namespace hal
{

class RealSense2IMUDevice;

class RealSense2IMUDriver : public IMUDriverInterface
{
  public:

    RealSense2IMUDriver(const std::vector<std::string>& ids);

    virtual ~RealSense2IMUDriver();

    /*bool Capture(CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    std::string GetDeviceProperty(const std::string& property) override;

    size_t NumChannels() const override;

    size_t Width(size_t index = 0) const override;

    size_t Height(size_t index = 0) const override;

    double MaxExposure(int channel = 0) const override;

    double MinExposure(int channel = 0) const override;

    double MaxGain(int channel = 0) const override;

    double MinGain(int channel = 0) const override;

    double Exposure(int channel = 0) override;

    void SetExposure(double exposure, int channel = 0) override;

    double Gain(int channel = 0) override;

    void SetGain(double gain, int channel = 0) override;

    double ProportionalGain(int channel = 0) const override;

    double IntegralGain(int channel = 0) const override;

    double DerivativeGain(int channel = 0) const override;

    double Emitter(int device) const;

    void SetEmitter(int device, double emitter) const;*/

    size_t NumDevices() const; 
	
    void RegisterIMUDataCallback(IMUDriverDataCallback callback);
    bool IsRunning() const override { return false; }
  protected:

    std::shared_ptr<RealSense2IMUDevice> Device(int channel);

    std::shared_ptr<const RealSense2IMUDevice> Device(int channel) const;

    static bool ValidDevice(rs2::device& device, const std::string& id);

    static bool ValidDevice(rs2::device& device);

  private:

    void Initialize();

    void CreateDevices();

    void CreateSelectedDevices();

    void CreateAllDevices();

    void SetChannelCount();

    void CreateMapping();

  protected:

    std::vector<std::shared_ptr<RealSense2IMUDevice>> devices_;

    std::vector<int> device_map_;

    std::vector<int> channel_map_;

    std::vector<std::string> ids_;

    int channel_count_;

    /*int width_;

    int height_;

    bool capture_color_;

    bool capture_depth_;

    bool capture_ir0_;

    bool capture_ir1_;

    int frame_rate_;*/
    IMUDriverDataCallback m_ImuCallback;
};

} // namespace hal
