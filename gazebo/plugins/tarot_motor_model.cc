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

#include <cmath>
#include <ignition/math/Pose3.hh>
#include <ignition/math.hh>
#include <vector>

#define N 3

namespace gazebo {

	class Tarot_Motor_Model : public ModelPlugin {

		// Pointer to the model.
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		//Subscriber pointer - Individual motors - declared as global variable.
		gazebo::transport::SubscriberPtr sub_motors;

		// Number of updates received
		public: int update_num = 0;
		
		//RPM : initial RPM is set to 0, received as Rev/Min, converted to rad/s
		public: double motor_velocities[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		public: double motor_rpms[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		
		// Variables to access tarot model in environment.
		public: physics::LinkPtr frame_;
		public: physics::LinkPtr fc_stack_;
		
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
			if ((_msg->motor_1()  / 9.549297 != motor_velocities[0]) || (_msg->motor_2()  / 9.549297 != motor_velocities[1])
				|| (_msg->motor_3() / 9.549297 != motor_velocities[2]) || (_msg->motor_4() / 9.549297 != motor_velocities[3])
				|| (_msg->motor_5() / 9.549297 != motor_velocities[4]) || (_msg->motor_6() / 9.549297 != motor_velocities[5])
				|| (_msg->motor_7() / 9.549297 != motor_velocities[6]) || (_msg->motor_8() / 9.549297 != motor_velocities[7])) {
				
				/*
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
						
				*/	
			}
			
			motor_velocities[0] = _msg->motor_1() / 9.549297;
			motor_velocities[1] = _msg->motor_2() / 9.549297;
			motor_velocities[2] = _msg->motor_3() / 9.549297;
			motor_velocities[3] = _msg->motor_4() / 9.549297;
			motor_velocities[4] = _msg->motor_5() / 9.549297;
			motor_velocities[5] = _msg->motor_6() / 9.549297;
			motor_velocities[6] = _msg->motor_7() / 9.549297;
			motor_velocities[7] = _msg->motor_8() / 9.549297;
			
			motor_rpms[0] = _msg->motor_1();
			motor_rpms[1] = _msg->motor_2();
			motor_rpms[2] = _msg->motor_3();
			motor_rpms[3] = _msg->motor_4();
			motor_rpms[4] = _msg->motor_5();
			motor_rpms[5] = _msg->motor_6();
			motor_rpms[6] = _msg->motor_7();
			motor_rpms[7] = _msg->motor_8();
			
		}


		//----------------------------------------
		// 			Load()
		// This function is called on the initalization of a Gazebo model that has this plugin.		
		//----------------------------------------
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

			// store pointer to the model.
			this->model = _parent;
			
			// Access frame link and fc_stack link. We'll apply forces to frame's CoG.
			frame_ = model->GetLink("frame");
			if (!frame_) {
				gzerr << "[Tarot_Motor_Model] Could not find frame link! Aborting.\n";
			}
			fc_stack_ = model->GetLink("fc_stack");
			if (!fc_stack_) {
				gzerr << "[Tarot_Motor_Model] Could not find fc_stack link! Aborting.\n";
			}
			
			//Create Protobuff Subscriber for Communication
			gazebo::transport::run();
			gazebo::transport::NodePtr node(new gazebo::transport::Node());
			node->Init();
			
			// Listen to tarot/motors for publishing an 8d vector representing rpm for each individual motor
			sub_motors = node->Subscribe("/tarot/motors", &Tarot_Motor_Model::cb_update_motors, this);
			gzdbg << "Subscribed to /tarot/motors for 8D Vector control." << std::endl;
			
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection =
				event::Events::ConnectWorldUpdateBegin(std::bind(&Tarot_Motor_Model::OnUpdate, this));

			gzdbg << "Tarot Motor Model Loaded. Motors Aligned in Spider Formation." << std::endl;

		}




		//----------------------------------------
		//			OnUpdate()
		// Called by the world update start event each simulation iteration.
		//----------------------------------------
		public: void OnUpdate() {
				
			// Each simulation iteration:
			//		* Set the angular velocity of each individual motor
			//		* Calculate resulting forces and torques of each motor
			//		* Apply Forces and Torques to the CoG of the UAV - the FC_Stack link
			
			this->SetVelocities();
			this->ApplyForcesAndTorques();
			
			

		}
		
		
		public: void SetVelocities() {
			
			this->model->GetJoint("prop_1_joint")->SetVelocity(0, motor_velocities[0]);
			this->model->GetJoint("prop_2_joint")->SetVelocity(0, motor_velocities[1] * -1);
			this->model->GetJoint("prop_3_joint")->SetVelocity(0, motor_velocities[2]);
			this->model->GetJoint("prop_4_joint")->SetVelocity(0, motor_velocities[3] * -1);
			this->model->GetJoint("prop_5_joint")->SetVelocity(0, motor_velocities[4]);
			this->model->GetJoint("prop_6_joint")->SetVelocity(0, motor_velocities[5] * -1);
			this->model->GetJoint("prop_7_joint")->SetVelocity(0, motor_velocities[6]);
			this->model->GetJoint("prop_8_joint")->SetVelocity(0, motor_velocities[7] * -1);
			
			
			
		}
		
		// --------------------------
		//		ApplyForcesAndTorques()
		// Function to read current angular velocities of the 8 motors and
		// Calculate the resulting thrust and torque acting on the parent frame.
		// Then applies the resulting forces and torques to the parent frame.
		// ** This version represents the "Spider" orientation for the Tarot T18.
		// ** For the SJM-1000, use Tarot_Motor_Model_Old
		// --------------------------
		
		public: void ApplyForcesAndTorques() {
			
			// Calculate Thrust
			double coef_t = 0.065;
			double T[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			double M[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			double T_total;
			double Phi;
			double The;
			double Psi;
			
			double g = 9.81;
			double m_t = 10.66;
			double l = 0.635;
			double d = 1.8503e-06; // rotor drag constant
			double b = 9.8419e-05; // thrust motor constant
			
			
			for (int i = 0; i < 8; i++) {
				// Aproach 1
				// thrust = coef_t * density of air * velocity^2 * propeller diameter (m) ^4
				//T[i] = coef_t * 1.225 * pow((motor_rpms[i] / 60.0), 2.0) * pow(0.47, 4.0);
				
				// Aproach 2 - Using thrust constant
				T[i] = b * pow((2*M_PI)*(motor_rpms[i] / 60.0), 2.0);
				
			}
			T_total = T[0] + T[1] + T[2] + T[3] + T[4] + T[5] + T[6] + T[7];
			T_total = round(T_total * 100) / 100;
			//gzdbg << "Total Thrust: " << T_total << "\n.";
			
			
			// Read Phi Theta and Psi Values.
			ignition::math::Pose3d pose = frame_->WorldPose();
			ignition::math::Quaterniond orientation_quat = pose.Rot();
			ignition::math::Vector3d orientation = orientation_quat.Euler();
			Phi = orientation.X();
			The = orientation.Y();
			Psi = orientation.Z();
			
			// Forces in the local frame
			double R_b_e[3][3] = 
			{	{cos(Psi)*cos(The), cos(Psi)*sin(The)*sin(Phi)-sin(Psi)*cos(Phi), cos(Psi)*sin(The)*cos(Phi)+sin(Psi)*sin(Phi)} ,
				{sin(Psi)*cos(The), sin(Psi)*sin(The)*sin(Phi)+cos(Psi)*cos(Phi), sin(Psi)*sin(The)*cos(Phi)-cos(Psi)*sin(Phi)} ,
				{-sin(The), cos(The)*sin(Phi), cos(The)*cos(Phi)}	};
			double R_b_e_tr[3][3];
			transpose(R_b_e, R_b_e_tr);
			
			
			//Create Column Vector by multiplying R_b_e_tr by column vector
			double Ftot_b[3];
			Ftot_b[0] = (R_b_e_tr[0][0] * 0) + (R_b_e_tr[0][1] * 0) + (R_b_e_tr[0][2]);
			Ftot_b[1] = (R_b_e_tr[1][0] * 0) + (R_b_e_tr[1][1] * 0) + (R_b_e_tr[1][2]);
			Ftot_b[2] = (R_b_e_tr[2][0] * 0) + (R_b_e_tr[2][1] * 0) + (R_b_e_tr[2][2]);
			Ftot_b[0] = Ftot_b[0] + 0;
			Ftot_b[1] = Ftot_b[1] + 0;
			Ftot_b[2] = Ftot_b[2] + T_total;
			
			ignition::math::Vector3d force;
			force = ignition::math::Vector3d(Ftot_b[0], Ftot_b[1], Ftot_b[2]);
			
			
			// Moments in the local frame
			// Moment of each rotor
			for (int i = 0; i < 8; i++) {
				// Aproach 1
				//M[i] = (2.138e-08) * pow(motor_rpms[i], 2) + (-1.279e-05) * motor_rpms[i];
				
				//Aproach 2 - Using Drag constant
				M[i] = d * pow((2*M_PI) * (motor_rpms[i] / 60.0) , 2.0);
			}
			
			// Torque generated by each rotor
			double Mx;
			double My;
			double Mz;
			
			double angsm = cos(M_PI/8);  // 22.5 degrees
			double anglg = cos(3*M_PI/8); // 67.5 degrees
			
			// Pitch (Mx) Forward and Backward Movement
			Mx = l * (T[0]*anglg + T[1]*angsm - T[2]*angsm - T[3]*anglg - T[4]*anglg
				- T[5]*angsm + T[6]*angsm + T[7]*anglg);
				
			// Roll (My) Left and Right Movement
			My = l * (-T[0]*angsm - T[1]*anglg - T[2]*anglg - T[3]*angsm + T[4]*angsm
				+ T[5]*anglg + T[6]*anglg + T[7]*angsm);
				

				
			// Yaw (Mz) CW and CCW Rotation
			Mz = M[0] - M[1] + M[2] - M[3] + M[4] - M[5] + M[6] - M[7];
			
			
			ignition::math::Vector3d torque;
			torque = ignition::math::Vector3d(Mx , My , Mz);
			
			// Add Force and Torque to Link frame
			fc_stack_->AddRelativeForce(force);
			fc_stack_->AddTorque(torque);
			
			//gzdbg << "Force: " << force.X() << " " << force.Y() << " " << force.Z() << "\n";
			
			
			
		}
		
		
		public: void transpose(double A[][N], double B[][N]) {
			int i, j;
			for (i = 0; i < N; i++)
				for (j = 0; j < N; j++)
					B[i][j] = A[j][i];
			}
		
		
		
		
		
		
		

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(Tarot_Motor_Model)

}
