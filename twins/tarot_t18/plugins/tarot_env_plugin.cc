

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/Base.hh>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <iomanip>
#include <functional>
#include <fcntl.h>
#include <cstdlib>
#include <unistd.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/vector8d.pb.h>

#include <vector>


//#include <boost/thread.hpp>


namespace gazebo {
	
class tarot_env : public WorldPlugin
{
	
	// Variables
	
	public: boost::thread callbackLoopThread;	//Main loop thread for the server.
	
	public: physics::WorldPtr world;	//Pointer to the world
	
	public: event::ConnectionPtr updateConnection;	//Pointer to event update connection
	
	public: std::mutex mutex;	//Controller update mutex
	
	public: int handle;		//Socket handle
	
	public: struct sockaddr_in remaddr;
	
	public: socklen_t remaddrlen;
	
	public: tarotPB::msgs::Vector8d cmd_vector;	//Command message holding 8d array of motor cmds
	
	int n = 8;
	public: std::vector<int> state{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	private: std::string cmdPubTopic;
	private: transport::NodePtr nodeHandle;
	
	// Communication channels to digital twin
	private: transport::PublisherPtr cmdPub;
	
	
	
	public: tarot_env() : WorldPlugin() {
		
		GOOGLE_PROTOBUF_VERIFY_VERSION;
		
		// Create UDP Socket
		this->handle = socket(AF_INET, SOCK_DGRAM, 0);
		int one = 1;
		setsockopt(this->handle, IPPROTO_TCP, TCP_NODELAY, 
			reinterpret_cast<const char *>(&one), sizeof(one));
		
		setsockopt(this->handle, SOL_SOCKET, SO_REUSEADDR, 
			reinterpret_cast<const char *>(&one), sizeof(one));
		
		fcntl(this->handle, F_SETFL,
			fcntl(this->handle, F_GETFL, 0) | O_NONBLOCK);
		
	}
	
	public: ~tarot_env() {
		gazebo::transport::fini();
		this->callbackLoopThread.join();
	}
	
	
	
	public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		
		// Create World Pointer, and Create and bind Socket addresses
		this->world = _world;
		this->LoadVars();
		
		// Create Gazebo Transport Node.
		this->nodeHandle = transport::NodePtr(new transport::Node());
		this->nodeHandle->Init();
		
		// Create a publisher to step the world and send an 8D vector to the esc_plugin
		this->cmdPub = this->nodeHandle->Advertise<tarotPB::msgs::Vector8d>("/tarot/motors");
		gzdbg << "Environment Publisher Created." << std::endl;

		// Force Pause because we drive the simulation steps
		this->world->SetPaused(true);
		
		this->callbackLoopThread =
			boost::thread(boost::bind(&tarot_env::LoopThread, this));
		
		
		// TODO: Use nodeHandle to subscribe to wherever we're gonna send the 12D Vector.
		
		
	}
	
	
	public: void LoadVars() {
		
		int port = 9002;
		gzdbg << "ENV Binding on port " << port << "\n";
		if (!this->Bind("127.0.0.1", port)) {
			gzerr << "Failed to bind with 127.0.0.1:" << port <<", aborting plugin.\n";
			return;
		}
		
		// Maybe need to Load digital twin model here?
		// Don't think this plugin needs to deal with the tarot itself. Only communicating
		// by publishing to topics.
		
	}
	
	
	public: bool Bind(const char *_address, const uint16_t _port) {
		
		struct sockaddr_in sockaddr;
		this->MakeSockAddr(_address, _port, sockaddr);
		
		if (bind(this->handle, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
			//shutdown(this->handle, 0);
			close(this->handle);
			return false;
		}
		return true;
	}
	
	
	public: void MakeSockAddr(const char *_address, const uint16_t _port,
		struct sockaddr_in &_sockaddr) {
		
		memset(&_sockaddr, 0, sizeof(_sockaddr));
		
		#ifdef HAVE_SOCK_SIN_LEN
			_sockaddr.sin_len = sizeof(_sockaddr);
		#endif
		
		_sockaddr.sin_port = htons(_port);
		_sockaddr.sin_family = AF_INET;
		_sockaddr.sin_addr.s_addr = inet_addr(_address);
		
	}
	
	
	// Loop thread: Enters a continuous loop waiting to receive an action over UDP
	public: void LoopThread() {
		
		while(1) {
			
			bool ac_received = this->ReceiveAction();
			
			if (!ac_received) {
				continue;
			}
			
			
			// Received an action from our python simulator
			// TODO: Handle a Reset command and a motor velocity set command.
			
			// Reset Command
			
			
			// Motor Command sent to esc_plugin
			this->cmdPub->Publish(this->cmd_vector);
			
			//Step the world
			this->world->Step(1);
			
			
			// Send state vector back through UDP
			//this->SendState();
		}
		
	}
	
	//--------------------------
	// Receive Action: Function to grab a message sent over UDP
	//--------------------------
	public: bool ReceiveAction() {
		
		unsigned int buf_size = 1024;
		char buf[buf_size];
		int recvSize;
		
		recvSize = recvfrom(this->handle, buf, buf_size , 0, (struct sockaddr *)&this->remaddr, 
				&this->remaddrlen);
		
		if (recvSize < 0) {
			return false;
		}
		
		
		std::string msg;
		msg.assign(buf, recvSize);
		// TODO: Here we assign the sent UDP message to a string to send via Protobuf.
		this->cmd_vector.ParseFromString(msg);
		//Then we return true that we've received a message
		return true;
		
	}
	
	
	public: void SendState() const {
		
		std::string buf = "0 0 0 0 0 0 0 0 0 0 0 0";

		::sendto(this->handle,
           buf.data(),
           buf.size(), 0,
		   (struct sockaddr *)&this->remaddr, this->remaddrlen);
		
	}
	
	
	
	
	
};

GZ_REGISTER_WORLD_PLUGIN(tarot_env)

}
