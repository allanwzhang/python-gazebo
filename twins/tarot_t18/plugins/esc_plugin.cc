#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <ignition/math/Vector3.hh>
#include <gazebo/msgs/vector8d.pb.h>

namespace gazebo {

	class ESCPlugin : public ModelPlugin {

		// Pointer to the model.
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		//Subscriber pointer - Individual motors - declared as global variable.
		gazebo::transport::SubscriberPtr sub_motors;

		// Number of updates received
		public: int update_num = 0;
		
		//RPM : (angular velocity) : initial RPM is set to 0, received as Rev/Min, converted to rad/s
		public: double vel_motor_1 = 0.0;
		public: double vel_motor_2 = 0.0;
		public: double vel_motor_3 = 0.0;
		public: double vel_motor_4 = 0.0;
		public: double vel_motor_5 = 0.0;
		public: double vel_motor_6 = 0.0;
		public: double vel_motor_7 = 0.0;
		public: double vel_motor_8 = 0.0;
		
		public: double vel_all_motors = 0.0;
		
		
		// Shared pointer object for 8D vector
		typedef const boost::shared_ptr<const tarotPB::msgs::Vector8d> Vector8dPtr;
		


		//----------------------------------------
		//				CB_UPDATE_MOTORS
		// Callback vector for updating all motors via a custom 8D Vector message.
		// This callback should be called every simulation step to simulate constantly changing velocities.
		// One subscriber on this channel, and the publisher should be the world plugin that publishes each
		// simulation step.
		//----------------------------------------
		public: void cb_update_motors(Vector8dPtr &_msg) {
			
			// Check if published Vector is different than current velocity
			if ((_msg->motor_1()  / 9.549297 != vel_motor_1) || (_msg->motor_2()  / 9.549297 != vel_motor_2)
				|| (_msg->motor_3() / 9.549297 != vel_motor_3) || (_msg->motor_4() / 9.549297 != vel_motor_4)
				|| (_msg->motor_5() / 9.549297 != vel_motor_5) || (_msg->motor_6() / 9.549297 != vel_motor_6)
				|| (_msg->motor_7() / 9.549297 != vel_motor_7) || (_msg->motor_8() / 9.549297 != vel_motor_8)) {
				
					gzdbg << "UPDATE ~ Received Array of motor velocities." << std::endl <<
						"\t[ " <<
						_msg->motor_1() << " " <<
						_msg->motor_2() << " " <<
						_msg->motor_3() << " " <<
						_msg->motor_4() << " " <<
						_msg->motor_5() << " " <<
						_msg->motor_6() << " " <<
						_msg->motor_7() << " " <<
						_msg->motor_8() << " ]" << std::endl;		
			}
			
			vel_motor_1 = _msg->motor_1() / 9.549297;
			vel_motor_2 = _msg->motor_2() / 9.549297;
			vel_motor_3 = _msg->motor_3() / 9.549297;
			vel_motor_4 = _msg->motor_4() / 9.549297;
			vel_motor_5 = _msg->motor_5() / 9.549297;
			vel_motor_6 = _msg->motor_6() / 9.549297;
			vel_motor_7 = _msg->motor_7() / 9.549297;
			vel_motor_8 = _msg->motor_8() / 9.549297;
			
		}


		//----------------------------------------
		// 			Load()
		// This function is called on the initalization of a Gazebo model that has this plugin.		
		//----------------------------------------
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

			//store pointer to the model.
			this->model = _parent;

			gazebo::transport::run();
			
			//Create Protobuff Subscriber for Communication
			gazebo::transport::NodePtr node(new gazebo::transport::Node());
			node->Init();
			
			// Listen to tarot/motors for publishing an 8d vector representing rpm for each individual motor
			sub_motors = node->Subscribe("/tarot/motors", &ESCPlugin::cb_update_motors, this);
			gzdbg << "Subscribed to /tarot/motors for 8D Vector control." << std::endl;
			
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection =
				event::Events::ConnectWorldUpdateBegin(std::bind(&ESCPlugin::OnUpdate, this));

			gzdbg << "Loaded ESC Plugin" << std::endl;

		}




		//----------------------------------------
		//			OnUpdate()
		// Called by the world update start event each simulation iteration.
		//----------------------------------------
		public: void OnUpdate() {
				
			// Each simulation iteration:
			//		* Set the angular velocity of each individual motor
			
			if (this->model->GetJoint("prop_1_joint")->GetVelocity(0) != vel_motor_1) {
				this->model->GetJoint("prop_1_joint")->SetVelocity(0, vel_motor_1);
			}
			if (this->model->GetJoint("prop_2_joint")->GetVelocity(0) != vel_motor_2 * -1) {
				this->model->GetJoint("prop_2_joint")->SetVelocity(0, vel_motor_2* -1);
			}
			if (this->model->GetJoint("prop_3_joint")->GetVelocity(0) != vel_motor_3) {
				this->model->GetJoint("prop_3_joint")->SetVelocity(0, vel_motor_3);
			}
			if (this->model->GetJoint("prop_4_joint")->GetVelocity(0) != vel_motor_4 * -1) {
				this->model->GetJoint("prop_4_joint")->SetVelocity(0, vel_motor_4 * -1);
			}
			if (this->model->GetJoint("prop_5_joint")->GetVelocity(0) != vel_motor_5) {
				this->model->GetJoint("prop_5_joint")->SetVelocity(0, vel_motor_5);
			}
			if (this->model->GetJoint("prop_6_joint")->GetVelocity(0) != vel_motor_6 * -1) {
				this->model->GetJoint("prop_6_joint")->SetVelocity(0, vel_motor_6 * -1);
			}
			if (this->model->GetJoint("prop_7_joint")->GetVelocity(0) != vel_motor_7) {
				this->model->GetJoint("prop_7_joint")->SetVelocity(0, vel_motor_7);
			}
			if (this->model->GetJoint("prop_8_joint")->GetVelocity(0) != vel_motor_8 * -1) {
				this->model->GetJoint("prop_8_joint")->SetVelocity(0, vel_motor_8 * -1);
			}
			
			/*
			this->model->GetJoint("prop_1_joint")->SetVelocity(0, vel_motor_1);
			this->model->GetJoint("prop_2_joint")->SetVelocity(0, vel_motor_2 * -1);
			this->model->GetJoint("prop_3_joint")->SetVelocity(0, vel_motor_3);
			this->model->GetJoint("prop_4_joint")->SetVelocity(0, vel_motor_4 * -1);
			this->model->GetJoint("prop_5_joint")->SetVelocity(0, vel_motor_5);
			this->model->GetJoint("prop_6_joint")->SetVelocity(0, vel_motor_6 * -1);
			this->model->GetJoint("prop_7_joint")->SetVelocity(0, vel_motor_7);
			this->model->GetJoint("prop_8_joint")->SetVelocity(0, vel_motor_8 * -1);
			*/

			update_num++;
			

		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ESCPlugin)

}
