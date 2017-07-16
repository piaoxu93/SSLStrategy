#ifndef __SSL_STRATEGY_H__
#define __SSL_STRATEGY_H__

#include <robokit/core/rbk_core.h>

class SSLStrategy : public rbk::NPluginInterface {
public:
	SSLStrategy();
	~SSLStrategy();
	void run();
	void loadFromConfigFile();
	void setSubscriberCallBack();
	void messageRawVisionCallBack(google::protobuf::Message*);
private:
	volatile bool m_recv_raw_vision;
};

RBK_INHERIT_PROVIDER(SSLStrategy,rbk::NPluginInterface,"1.0.0");

#endif