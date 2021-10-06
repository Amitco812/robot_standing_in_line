#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo
{
  class MoveCup : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MoveCup::OnUpdate, this));
      this->old_secs = ros::Time().toSec();
      ROS_WARN("Loaded MoveCup Plugin with parent...%s", this->model->GetName().c_str());
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
        double curr_time = ros::Time::now().toSec();
        if(this->old_secs == 0.0){
          this->old_secs = curr_time;
        }
        double delta = curr_time - this->old_secs;
        if(delta > 50.0 && this->move_cup){
          this->model->SetWorldPose(math::Pose(0,0,0,0,0,0));
          this->move_cup=false;
        }
    }
    
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //time memory
    double old_secs;

    bool move_cup=true;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveCup)
}