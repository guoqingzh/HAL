/*
   \file NodeCamDriver.cpp
 */
#include "NodeCamDriver.h"
#include "stdlib.h"
#include <HAL/Devices/DeviceException.h>

namespace hal
{

NodeCamDriver::NodeCamDriver(const Uri& uri)
{
  m_sDeviceId = uri.properties.Get<std::string>("id","NodeCam");
  m_sSimNodeName = uri.properties.Get<std::string>("sim","LocalSim");

  //Default value is device Id as id will be used when name is not present.
  m_sDeviceName = uri.properties.Get<std::string>("name", m_sDeviceId);
  m_sTopic = m_sSimNodeName + "/" + m_sDeviceName;

  // Initialize and create the node, if the node is initialized then register
  // the camera in simba.
  bool initDone = InitNode() && RegisterInHost(uri);
  if(!initDone) {
    throw DeviceException("Could Not initialize the node or register the "
                          "camera in Simulator, messages printed above should"
                          " be helpful.");
  }

  std::cout<<"init NodeCam success"<<std::endl;
}


NodeCamDriver::~NodeCamDriver()
{

}

// capture images from host
bool NodeCamDriver::Capture( pb::CameraMsg& vImages )
{
  CamMsg         Msg;
  CamMsg         TryMsg;

  // here we use max try to avoid infinite wait
  int iMaxTry=5;
  bool bSuccessFlag = false;
  while (bSuccessFlag==false && iMaxTry>0)
  {
    std::cout<<"Going in Battle"<<std::endl;
    if(m_Node.receive(m_sTopic, Msg)==true )//&& TryMsg.time_step() == m_nTimeStep+1)
    {
      bSuccessFlag = true;
    }
    else
    {
      iMaxTry--;
    }
    usleep(1000);
    std::cout<<"Battle's over"<<std::endl;
  }

  if(bSuccessFlag==false)
  {
    std::cout <<"[NodeCamDriver/Capture] Fail. Did not receive images form "
              <<m_sTopic<<std::endl;
    return false;
  }
  else
    std::cout<<"Got an Image."<<std::endl;

  m_nChannels = Msg.size();

  for(unsigned int ii = 0; ii != m_nChannels; ii++ )
  {
    const ImageMsg& Img = Msg.image(ii);

    if(Img.image_type()==1)//------------------------Gray
    {
      pb::ImageMsg* pbImg = vImages.add_image();
      pbImg->set_timestamp( m_nTimeStep);
      pbImg->set_width( m_nImgWidth );
      pbImg->set_height( m_nImgHeight );
      pbImg->set_type(pb::PB_UNSIGNED_SHORT);
      pbImg->set_format(pb::PB_LUMINANCE);
      pbImg->set_data( Img.image().c_str(), m_nImgWidth * m_nImgHeight );
    }
    else if(Img.image_type()==2)//-------------------RGB
    {
      pb::ImageMsg* pbImg = vImages.add_image();
      pbImg->set_timestamp( m_nTimeStep);
      pbImg->set_width( m_nImgWidth );
      pbImg->set_height( m_nImgHeight );
      pbImg->set_type(pb::PB_UNSIGNED_BYTE);
      pbImg->set_format(pb::PB_RGB);
      pbImg->set_data( Img.image().c_str(), m_nImgWidth * m_nImgHeight *3);
    }
    else if(Img.image_type()==5)//------------------Depth
    {
      pb::ImageMsg* pbImg = vImages.add_image();
      pbImg->set_timestamp( m_nTimeStep);
      pbImg->set_width( m_nImgWidth );
      pbImg->set_height( m_nImgHeight );
      pbImg->set_type(pb::PB_FLOAT);
      pbImg->set_format(pb::PB_LUMINANCE);
      pbImg->set_data( Img.image().c_str(), m_nImgWidth * m_nImgHeight *4);
    }
  }

  return true;
}

std::string NodeCamDriver::GetDeviceProperty(const std::string& sProperty)
{
  // TODO add property with suffix of camera (ie. DepthBaseline0, DepthBaseline1)
  // and return correct vector info
  //    if(sProperty == hal::) {
  //        return std::to_string( m_DepthBaselines[0] );
  //    }
  //    if(sProperty == hal::DeviceDepthFocalLength) {
  //        return std::to_string( m_DepthFocalLengths[0] );
  //    }
  return std::string();
}

size_t NodeCamDriver::NumChannels() const
{
  return  m_nChannels;
}

size_t NodeCamDriver::Width( size_t /*idx*/ ) const
{
  return m_nImgWidth;
}

size_t NodeCamDriver::Height( size_t /*idx*/ ) const
{
  return m_nImgHeight;
}


bool NodeCamDriver::InitNode()
{
  m_Node.set_verbocity(2); // make some noise on errors
  if(m_Node.init(m_sDeviceName)==false)
  {
    std::cerr <<"[NodeCamDriver] Cannot init NodeCam '"<<m_sDeviceName<<"'"
              <<std::endl;
    return false;
  }

  return true;
}

// register NodeCam in Host
bool NodeCamDriver::RegisterInHost(const hal::Uri& uri)
{
  RegisterNodeCamReqMsg mReq;
  RegisterNodeCamRepMsg mRep;

  //mReq.set_uri(uri.ToString()); // This is the future.

  // TODO: Correct names.
  m_sDeviceName = "RGB_LCamera@Robot@Proxy1";
  m_sTopic = m_sSimNodeName + "/" + m_sDeviceName;
  mReq.set_uri(m_sDeviceName);
  int nTries=0;
  while(nTries < 5 &&
       !m_Node.call_rpc(m_sSimNodeName, "RegsiterCamDevice", mReq, mRep, 100))
  {
    std::cerr << "[NodeCamDriver/RegisterInHost] Error Call Rpc method of '"
              << m_sSimNodeName
              << "'. Cannot connect to host!! Please make sure it is running!"
              << std::endl;
    sleep(1);
    nTries++;
  }

  if(mRep.regsiter_flag()==1)
  {
    m_nTimeStep =  mRep.time_step();
    m_nChannels = mRep.channels();
    m_nImgHeight = mRep.width();
    m_nImgWidth = mRep.height();

    if( m_Node.subscribe(m_sTopic) == false )
    {
      std::cerr << "[NodeCamDriver/RegisterInHost] Error subscribing to '"
                << m_sTopic
                << "'. Please make sure "<<m_sTopic<<" is running !!"
                << std::endl;
      return false;
    }
    else
    {
      std::cout << "[NodeCamDriver/RegisterInHost] Subscribe to topic '"
                << m_sTopic
                << "' success!"<<std::endl;
    }
  }
  else
  {
    std::cerr << "[NodeCamDriver/RegisterInHost] Cannot register RPG to "
              << m_sSimNodeName
              << std::endl;
    return false;
  }

  return true;
}

}

