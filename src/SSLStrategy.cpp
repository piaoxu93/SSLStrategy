#include "SSLStrategy.h"
RBK_INHERIT_SOURCE(SSLStrategy)
#include <WinSock2.h>
#include <iostream>
#include <string>
#include <fstream>
#include <WorldModel.h>
#include "DecisionModule.h"
#include "ActionModule.h"
#include <tinyxml/ParamReader.h>
#include <CtrlBreakHandler.h>
#include <TimeCounter.h>
#include <GDebugEngine.h>
#include "bayes/MatchState.h"
#include "gpuBestAlgThread.h"
#include "Global.h"
#include "DefendUtils.h"
#include "Compensate.h"
#include "src_debug.pb.h"
#include "referee.pb.h"
#include "src_cmd.pb.h"
#include "src_rawvision.pb.h"
#include "VisionReceiver4rbk.h"
#include <robokit/core/model.h>
#include <robokit/core/rbk_config.h>

#if defined(RBK_SYS_WINDOWS) && _MSC_VER <= 1600
#include <boost/chrono.hpp>
#else
#include <chrono>
#endif

namespace { 
	bool VERBOSE_MODE = true;
	void debugMessageSend();
}
rbk::protocol::Debug_Msgs guiDebugMsgs;
std::string CParamReader::_paramFileName = "zeus2005";

//extern CEvent *visionEvent;
//extern CMutex *decisionMutex;
CMutex* _best_visiondata_copy_mutex = 0;
CMutex* _value_getter_mutex = 0;

CUsecTimer _usecTimer;
CUsecTimer _usecTime2;

using Param::Latency::TOTAL_LATED_FRAME;
int gui_port = 20001;
bool gui_pause = false;
bool wireless_off = false;
bool send_debug_msg = false;
//CThreadCreator* gui_recv_thread = 0;

//static void do_gui_send(int cycle);

//DWORD THREAD_CALLBACK do_gui_recv(LPVOID lpParam);

SSLStrategy::SSLStrategy(void)
{
	m_recv_raw_vision = false;
}

SSLStrategy::~SSLStrategy(void)
{
}

void SSLStrategy::messageRawVisionCallBack(google::protobuf::Message*)
{
	m_recv_raw_vision = true;
}

void SSLStrategy::run(){
#if defined(RBK_SYS_WINDOWS) && _MSC_VER <= 1600
	boost::chrono::time_point<boost::chrono::steady_clock> tp1;
	boost::chrono::time_point<boost::chrono::steady_clock> tp2;
	tp1 = boost::chrono::steady_clock::now();
#else
	std::chrono::time_point<std::chrono::steady_clock> tp1;
	std::chrono::time_point<std::chrono::steady_clock> tp2;
	tp1 = std::chrono::steady_clock::now();
#endif

	PARAM_READER->readParams();

	initializeSingleton();
	ofstream file;
	file.open("./data/ssl/frequence.txt",ios::out);
	CCtrlBreakHandler breakHandler;
	COptionModule option;
	vision->registerOption(&option);
	CDecisionModule decision(&option, vision);
	CActionModule action(&option, vision, &decision);
	WORLD_MODEL->registerVision(vision);
	MATCH_STATE->initialize(&option,vision);

	_best_visiondata_copy_mutex = new CMutex;
	_value_getter_mutex = new CMutex;
	GPUBestAlgThread::Instance()->initialize(VISION_MODULE);
	VisualInfoT visionInfo;
	RefRecvMsg refRecvMsg;
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
	bool stop = false;

	double usedtime = 0.0f;
	double totalusedtime = 0.0f;
	while (true) {
		//bool is_simulation;
		//rbk::Config::Instance()->get("simulation", is_simulation);
		//std::cout<<is_simulation<<std::endl;
		if(breakHandler.breaked() < 1 && m_recv_raw_vision){
			m_recv_raw_vision = false;
			usedtime = 0.0f;
			totalusedtime = 0.0f;	

			// change 2017.7.9 seperate each thread so we don't need event or mutex for vision and decision modules.
			//visionEvent->wait();
			//decisionMutex->lock();
			// vision and referee --> change for rbk
			if (! VisionReceiver4rbk::Instance()->getVisionInfo(&option,visionInfo, refRecvMsg)) {
				Sleep(2);
				continue;
			}

			if (VERBOSE_MODE) {
				if (visionInfo.cycle % 2 ==0) {
					_usecTime2.start();
				} else {
					_usecTime2.stop();					
					if (file.is_open()) {
						file<<" Frequence : "<<visionInfo.cycle<<" : "<<_usecTime2.time()/1000.0f<<"\n";
					}						
				}
				_usecTimer.start();
				vision->SetRefRecvMsg(refRecvMsg);
				vision->SetNewVision(visionInfo);

				_usecTimer.stop();

				if (visionInfo.cycle%10 == 0) {
					usedtime = _usecTimer.time()/1000.0f;
					totalusedtime += usedtime;
				}
			} else {
				vision->SetRefRecvMsg(refRecvMsg);
				vision->SetNewVision(visionInfo);
			}

			if (0 == breakHandler.breaked()) {
				stop = true;
			}
			if (breakHandler.halted() || gui_pause) {
				decision.DoDecision(true);
			} else {
				decision.DoDecision(stop);
			}

			if (! wireless_off) {
				action.sendAction(visionInfo.ourRobotIndex);
			} else {
				action.sendNoAction(visionInfo.ourRobotIndex);
			}
			//decisionMutex->unlock();	

			// new addition for SSLGuiDugger plugin
			debugMessageSend();

#if defined(RBK_SYS_WINDOWS) && _MSC_VER <= 1600
			tp2 = boost::chrono::steady_clock::now();
			auto delt_time = boost::chrono::duration_cast<boost::chrono::milliseconds>((tp2 - tp1)).count();
			tp1 = boost::chrono::steady_clock::now();
#else
			tp2 = std::chrono::steady_clock::now();
			auto delt_time = std::chrono::duration_cast<std::chrono::milliseconds>((tp2 - tp1)).count();
			tp1 = std::chrono::steady_clock::now();
#endif
			if (delt_time < 1000.0/60) {
				// Ë¯Ãß£¬È¥µôÔëÉùÍ¼Ïñ£¬±£Ö¤ÎÈ¶¨µÄÍ¼ÏñÖ¡ÂÊ
				Sleep(1000.0/60 - delt_time);	
				//std::cout<<cycle<<'\t'<<1000.0/Param::Vision::FRAME_RATE-delt_time<<std::endl;
			}

			if (stop) {
				Sleep(100);
				break;
			}	
		}
	}
	
	if (file.is_open()) {
		file.close();
	}
	action.stopAll();
	std::cout << "system exits, press any key to continue..." << std::endl;
	getchar();
}
void SSLStrategy::loadFromConfigFile(){

}
void SSLStrategy::setSubscriberCallBack(){
	setTopicCallBack<rbk::protocol::SRC_RawVision>(&SSLStrategy::messageRawVisionCallBack,this);
	createPublisher<rbk::protocol::Debug_Msgs>();
	createPublisher<rbk::protocol::SRC_Cmd>();
	setTopicCallBack<SSL_Referee>();
	setTopicCallBack<rbk::protocol::Robots_Status>();
}
namespace{
	void debugMessageSend(){
		GRBKHandle::Instance()->publishTopic(guiDebugMsgs);
		guiDebugMsgs.Clear();
		return;
	}
}
//static void do_gui_send(int cycle){
//	//if (send_debug_msg) {
//	//	net_gdebugs new_msgs;
//	//	new_msgs.totalnum = 0;
//	//	new_msgs.totalsize = 0;
//	//	memset(new_msgs.msg, 0, 500*sizeof(net_gdebug));
//	//	while(!GDebugEngine::Instance()->is_empty()){
//	//		net_gdebug new_msg = GDebugEngine::Instance()->get_queue_front();
//	//		new_msgs.totalsize = vision->Cycle();
//	//		new_msgs.msg[new_msgs.totalnum] = new_msg;
//	//		GDebugEngine::Instance()->pop_front();
//	//		new_msgs.totalnum++;
//	//		if(new_msgs.totalnum >= 500){
//	//			cout<<"!!!!!!!!!To Many Debug Message To Client!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
//	//		}
//	//	}
//	//	gui_socket.sendData(gui_port, (char*)(&new_msgs), 8+sizeof(net_gdebug)*new_msgs.totalnum, "127.0.0.1");
//	//}
//	return;
//}
//
//DWORD THREAD_CALLBACK do_gui_recv(LPVOID lpParam)
//{
//	while (true) {
//		char msg[net_gui_in_maxsize];
//		if (gui_socket.receiveData(msg, net_gui_in_maxsize)) {
//			net_gcommand* ngc = (net_gcommand*)msg;
//			switch (ngc->msgtype) 
//			{
//			case NET_GUI_CONTROL:
//				{
//					if ( strcmp(ngc->cmd, "r") == 0 )
//						gui_pause = false;
//					else if ( strcmp(ngc->cmd, "p") == 0 )
//						gui_pause = true;
//					else if ( strcmp(ngc->cmd, "wireless_on") == 0 )
//						wireless_off = false;
//					else if ( strcmp(ngc->cmd, "wireless_off") == 0 )
//						wireless_off = true;
//					else if ( strcmp(ngc->cmd, "strategy_on") == 0 )
//						send_debug_msg = true;
//					else if ( strcmp(ngc->cmd, "strategy_off") == 0 )
//						send_debug_msg = false;
//					break;
//				}
//			default:
//				break;
//			}
//		}
//		Sleep(100);
//	}
//}