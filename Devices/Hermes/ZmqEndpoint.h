#ifndef ZMQENDPOINT_H
#define ZMQENDPOINT_H

enum ZmqEndpointType
{
    ZMQ_EP_PUBLISH = 0,
    ZMQ_EP_SUBSCRIBE = 1,
    ZMQ_EP_REQUEST = 2,
    ZMQ_EP_REPLY = 3
};

class ZmqEndpoint {
	public:
		ZmqEndpoint();

	private:
		ZmqEndpointType 	m_eSocketType;
		zmq::socket_t*   	m_pSocket;
};

#endif // ZMQENDPOINT_H
