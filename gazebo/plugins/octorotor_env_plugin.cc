

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
#include <ignition/math.hh>


//#include <boost/thread.hpp>


namespace gazebo {
	
class Octorotor_Env : public WorldPlugin
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
	
	
	// State vector: Sent back over UDP so that Python models know what is happening.
	// 			Vector Values:
	//		[ pos_x , pos_y , pos_z , 
	//		  lin_vel_x , lin_vel_y , lin_vel_z , 
	//		  att_x , att_y , att_z ,
	//		  ang_vel_x , ang_vel_y , ang_vel_z ]
	public: std::vector<double> state_vector =
						{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	
	// Communication channels to digital twin
	private: transport::NodePtr nodeHandle;
	private: transport::PublisherPtr cmdPub;
	
	
	// Variables to access vehicle model in environment.
	public: std::string link_name;
	public: physics::LinkPtr link_;
	
	public: std::string model_name;
	public: physics::ModelPtr model_;
	
	
	
	public: Octorotor_Env() : WorldPlugin() {
		
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
	
	public: ~Octorotor_Env() {
		gazebo::transport::fini();
		this->callbackLoopThread.join();
	}
	
	
	
	public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		
		// Create World Pointer, and Create and bind Socket addresses
		this->world = _world;
		this->LoadVars(_sdf);
		
		// Create Gazebo Transport Node.
		this->nodeHandle = transport::NodePtr(new transport::Node());
		this->nodeHandle->Init();
		
		// Create a publisher to step the world and send an 8D vector to the esc_plugin
		this->cmdPub = this->nodeHandle->Advertise<tarotPB::msgs::Vector8d>("/tarot/motors");
		gzdbg << "Environment Publisher Created." << std::endl;

		// Force Pause because we drive the simulation steps
		this->world->SetPaused(true);
		
		this->callbackLoopThread =
			boost::thread(boost::bind(&Octorotor_Env::LoopThread, this));
		
		
	}
	
	
	public: void LoadVars(sdf::ElementPtr _sdf) {
		
		int port = 9002;
		gzdbg << "ENV Binding on port " << port << "\n";
		if (!this->Bind("127.0.0.1", port)) {
			gzerr << "Failed to bind with 127.0.0.1:" << port <<", aborting plugin.\n";
			return;
		}
		
		
		// plugin needs to access Tarot in order to record state values.
		if (_sdf->HasElement("modelName")) {
			model_name = _sdf->GetElement("modelName")->Get<std::string>();
		}
		else {
			gzerr << "[Octorotor_Env] Please specify the name of the Octorotor model.\n";
		}
		
		
		// Now get a pointer to the model
		model_ = this->world->ModelByName(model_name);
		if (!model_) {
			gzerr << "Could not access model " << model_name << " from world.\n";
		} 
		else {
			gzdbg << "Accessed Model: " << model_name << ".\n";
		}
		
		// Now access the FC_stack on the model.
		// All states will be recorded from this link as is the case in real flights.
		link_ = model_->GetLink("fc_stack");
		if (!link_) {
			gzerr << "[Octorotor_Env] Could not find Fc_Stack Link on " << model_name << ".\n";
		}
		else {
			gzdbg << "Accessed  Link: fc_stack from model: " << model_name << ".\n";
		}
		
		
		//Call SendState() once to set up connection
		this->SendState();
		
		
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
			this->SendState();
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
	
	
	public: int counter = 0;
	
	public: void SendState() {
		
		std::string buf = "0 0 0 0 0 0 0 0 0 0 0 0";
		
		// ~~Sensors~~
		
		// First, position
		ignition::math::Pose3d pose = link_->WorldPose();
		ignition::math::Vector3d position = pose.Pos();
		
		// Linear Velocity
		ignition::math::Vector3d lin_vel = link_->RelativeLinearVel();
		
		// Attitude
		// Use Pose3d object to get Rot() quaternion.
		// Then convert quaternion into Euler angles and send Euler angles back to controller
		ignition::math::Quaterniond orientation_quat = pose.Rot();
		ignition::math::Vector3d orientation = orientation_quat.Euler();
		
		
		// Angular Velocity
		ignition::math::Vector3d angular_vel = link_->RelativeAngularVel();
		
		
		// Fill State Vector with values
		this->state_vector.at(0) = position.X();
		this->state_vector.at(1) = position.Y();
		this->state_vector.at(2) = position.Z();
		
		this->state_vector.at(3) = lin_vel.X();
		this->state_vector.at(4) = lin_vel.Y();
		this->state_vector.at(5) = lin_vel.Z();
		
		this->state_vector.at(6) = orientation.X();
		this->state_vector.at(7) = orientation.Y();
		this->state_vector.at(8) = orientation.Z();
		
		this->state_vector.at(9) = angular_vel.X();
		this->state_vector.at(10) = angular_vel.Y();
		this->state_vector.at(11) = angular_vel.Z();
		
		
		// Now copy state_vector into buffer string to send over UDP
		char space = ' ';
		std::string buffer = std::to_string(this->state_vector[0]);
		for (int i = 1; i < 12; i++) {
			buffer.push_back(space);
			buffer.append(std::to_string(this->state_vector[i]));
		}
		
		
		//gzdbg << "String to send back: " << buffer << "\n";
		
		
		// Send buffer string through UDP socket as raw bytes
		::sendto(this->handle,
           buffer.data(),
           buffer.size(), 0,
		   (struct sockaddr *)&this->remaddr, this->remaddrlen);
		   
		
		
		
	}
	
	
	
	
	
};

GZ_REGISTER_WORLD_PLUGIN(Octorotor_Env)

}
