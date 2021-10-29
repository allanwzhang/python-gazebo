#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{

class WorldPluginTutorial : public WorldPlugin
{

    public: WorldPluginTutorial() : WorldPlugin()
    {
        gzmsg << "Hello World!\n";
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        world = _world;
        transport::run();
        node = transport::NodePtr(new transport::Node());
        node->Init();
        node->Advertise<msgs::Any>("~/sub");
        sub = node->Subscribe("~/sub", &WorldPluginTutorial::printer_cls, this);

        msgs::Any msg;
        msg.set_type(msgs::Any::STRING);
        msg.set_string_value("Gazebo docs are absolutely imbecilical\n");
        gzmsg << msg.SerializeAsString() << std::endl;
        // gz topic -p /gazebo/box-world/sub -m 'string_value:"yolo", type:STRING'
    }

    public: void printer_cls(ConstAnyPtr &msg)
    {
        if (msg->type() == msg->INT32)
        {
            gzmsg << "Got an INT32! " << msg->int_value() << std::endl;
        }
        else if (msg->type() == msg->STRING)
        {
            std::string val = msg->string_value();
            gzmsg << val << std::endl;
            if (val=="reset")
            {
                world->SetPaused(true);
                world->ResetTime();
                world->ResetEntities(physics::Base::BASE);
                // physics::ModelPtr box = world->ModelByName("box");
                // box->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
                // box->SetWorldPose(ignition::math::Pose3d(
                //     ignition::math::Vector3d(0, 0, 0.5),
                //     ignition::math::Quaterniond(0, 0, 0)));
                // world->Reset();
                world->SetPaused(false);
            }
        }
    }

    public: ~WorldPluginTutorial()
    {
		transport::fini();
    }

    private:
        transport::NodePtr node;
        transport::SubscriberPtr sub;
        physics::WorldPtr world;
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);

}
