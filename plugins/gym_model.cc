#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>



namespace gazebo
{
class GymModelPlugin : public ModelPlugin
{
    public: GymModelPlugin() : ModelPlugin()
    {
        gzmsg << "Hello Model!\n";
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        model = _parent;
        world = model->GetWorld();
        links = model->GetLinks();
        mainLink = model->GetLink("main");
        lastSec = 0;

        updateEvent = event::Events::ConnectWorldUpdateBegin(std::bind(&GymModelPlugin::onUpdate, this));
    }

    public: void onUpdate()
    {
        currSec = floor(world->SimTime().Float());
        if (currSec > lastSec + 2)
        {
            gzmsg << currSec    << "\t" << mainLink->WorldLinearVel() 
                                << "\t" << mainLink->WorldPose().Pos() << std::endl;
            lastSec = currSec;
        }
        mainLink->SetForce(ignition::math::Vector3d(0.1, 0, 0));
    }


    private:
        physics::WorldPtr world;
        physics::ModelPtr model;
        physics::Link_V links;
        physics::LinkPtr mainLink;
        event::ConnectionPtr updateEvent;
        int lastSec, currSec;
};
GZ_REGISTER_MODEL_PLUGIN(GymModelPlugin)
}