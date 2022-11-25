#pragma once

#include <librealsense2/rs.hpp>
#include <HAL/IMU/IMUDriverInterface.h>
#include <utility>
#include <Eigen/Eigen>
#include <deque>
namespace hal
{

typedef std::pair<rs2_stream, int> stream_index_pair;
const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

class RealSense2IMUDevice 
{
  public:

    RealSense2IMUDevice(rs2::device& device);

    virtual ~RealSense2IMUDevice();

    /*bool Capture(CameraMsg& images) override;

    std::shared_ptr<CameraDriverInterface> GetInputDevice() override;

    std::string GetDeviceProperty(const std::string& property) override;
    */
    size_t NumChannels() const;
    /*
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

    double Emitter() const;

    void SetEmitter(double emitter) const;*/

    void RegisterIMUDataCallback(IMUDriverDataCallback callback);
  protected:

    /*void EnableAutoExposure(int channel);

    void DisableAutoExposure(int channel, double exposure);

    void CaptureInfraredStream(int index, CameraMsg& images);

    void CaptureColorStream(CameraMsg& images);

    void CaptureDepthStream(CameraMsg& images);

    bool IsColorStream(int channel) const;*/

    const rs2::sensor& GetSensor(int channel) const;

    rs2::sensor& GetSensor(int channel);


  private:

        class CimuData
        {
            public:
                CimuData() : m_time_ns(-1) {};
                CimuData(const stream_index_pair type, Eigen::Vector3d data, double time):
                    m_type(type),
                    m_data(data),
                    m_time_ns(time){};
                bool is_set() {return m_time_ns > 0;};
            public:
                stream_index_pair m_type;
                Eigen::Vector3d m_data;
                double          m_time_ns;
        };


    void Initialize();

    
    void CreateSerialNumber();

    void CreatePipeline();

    void ConfigurePipeline();

    void CreateConfiguration();

    hal::ImuMsg CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

    void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<hal::ImuMsg>& imu_msgs);

    /*void ConfigureInfraredStream(int index);

    void ConfigureColorStream();

    void ConfigureDepthStream();*/

    void ConfigureAccelStream();
    void ConfigureGyroStream();

    void CreateSensors();

    void CreateStreams();

    /*void CreateInfraredStream(int index);

    void CreateColorStream();

    void CreateDepthStream();*/

  protected:

    std::unique_ptr<rs2::pipeline> pipeline_;

    std::unique_ptr<rs2::config> configuration_;

    std::vector<rs2::motion_stream_profile> streams_;

    //rs2::sensor depth_sensor_;

    //rs2::sensor color_sensor_;

    rs2::sensor motion_sensor_;

    rs2::frameset frameset_;

    rs2::device device_;

    std::string serial_string_;

    uint64_t serial_number_;

    IMUDriverDataCallback m_ImuCallback;

    /*int width_;

    int height_;

    bool capture_color_;

    bool capture_depth_;

    bool capture_ir0_;

    bool capture_ir1_;

    int frame_rate_;

    double exposure_;

    double gain_;

    double emitter_;*/
};

} // namespace hal
