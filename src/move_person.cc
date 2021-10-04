#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo
{
  class MovePerson : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MovePerson::OnUpdate, this));
      this->AddPoses(); //add all poses
      this->AddEnds(); //add all end points
      this->FindMyPose();
      this->old_secs = ros::Time().toSec();
      ROS_WARN("Loaded MovePerson Plugin with parent...%s", this->model->GetName().c_str());
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if(this->next_point >= -1){
        double curr_time = ros::Time::now().toSec();
        if(this->old_secs == 0.0){
          this->old_secs = curr_time;
        }
        double delta = curr_time - this->old_secs;
        
        if(to_wait){
          if(delta > this->waiting_time){
            ROS_WARN("need to walk");
            this->to_wait = !(this->to_wait);
            this->old_secs = curr_time;
            //here goes the logic for velocity in the right direction (heading to poses[next_point])
            math::Pose heading_to = this->next_point >= 0 ? poses[this->next_point] : this->my_end;
            math::Pose current_pos = this->model->GetWorldPose();
            double x_vel = (heading_to.pos.x - current_pos.pos.x)/(this->walking_time);
            double y_vel = (heading_to.pos.y - current_pos.pos.y)/(this->walking_time);
            ROS_WARN("x: %f , y: %f",x_vel,y_vel);
            this->model->SetLinearVel(ignition::math::Vector3d(x_vel, y_vel, 0));
          }
        }else{
          if(delta > this->walking_time){
            if(this->next_point > -1){
              this->model->SetWorldPose(poses[this->next_point]);
            }
            this->next_point--;
            ROS_WARN("need to wait, nextPoint: %d",this->next_point);
            this->to_wait = !(this->to_wait);
            this->old_secs = curr_time;
            this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
          }
        }
      }else if (to_wait){
        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      }
          
    }

    private: void AddPoses(){
      double zero=0.0;
      
      math::Pose p_barista = math::Pose(1.4,6.0,zero,zero,zero,-1.6);
      this->poses[0]=p_barista;
      math::Pose p2 = math::Pose(2.25,6.4,zero,zero,zero,-1.6);
      this->poses[1]=p2;
      math::Pose p3 = math::Pose(3.0,6.4,zero,zero,zero,-1.6);
      this->poses[2]=p3;
      math::Pose p4 = math::Pose(3.6,5.6,zero,zero,zero,-1.8);
      this->poses[3]=p4;
      math::Pose p5 = math::Pose(4.1,5.25,zero,zero,zero,-2);
      this->poses[4]=p5;
      math::Pose p6 = math::Pose(4.5,4.7,zero,zero,zero,-2.5);
      this->poses[5]=p6;
      math::Pose p7 = math::Pose(5.1,3.94,zero,zero,zero,-3);
      this->poses[6]=p7;

    }

    private: void AddEnds(){
      double zero=0.0;
      double x=-1.88;
      double y=-7.56;
      double delta=1.2;
      math::Pose p_end1 = math::Pose(x,y,zero,zero,zero,zero);
      this->ends[0]=p_end1;
      math::Pose p_end2 = math::Pose(x,y+delta,zero,zero,zero,zero);
      this->ends[1]=p_end2;
      math::Pose p_end3 = math::Pose(x,y+2*delta,zero,zero,zero,zero);
      this->ends[2]=p_end3;
      math::Pose p_end4 = math::Pose(x,y+3*delta,zero,zero,zero,zero);
      this->ends[3]=p_end4;
      math::Pose p_end5 = math::Pose(x,y+4*delta,zero,zero,zero,zero);
      this->ends[4]=p_end5;
      math::Pose p_end6 = math::Pose(x,y+5*delta,zero,zero,zero,zero);
      this->ends[5]=p_end6;
      math::Pose p_end7 = math::Pose(x,y+6*delta,zero,zero,zero,zero);
      this->ends[6]=p_end7;

    }

    private: void FindMyPose(){
      math::Pose my_pos = this->model->GetWorldPose();
      //only for the people in line
      for(int i=0;i<7;i++){
        math::Pose curr = poses[i];
        if(curr.pos.x == my_pos.pos.x && curr.pos.y == my_pos.pos.y){
          ROS_WARN("parent %s found position", this->model->GetName().c_str());
          this->next_point = i-1;
          this->my_end = ends[i];
          break;
        }
        
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //time memory
    double old_secs;

    //waiting_time
    double waiting_time = 5.0;

    //walking_time
    double walking_time = 3.0;

    bool to_wait = true;

    // 0 - first one in line etc ..
    //poses of people in the line (9 = line length + end point)
    math::Pose poses[7];

    //next target point to head to
    int next_point = -2;

    //end points for all models
    math::Pose ends[7];

    //own model end point
    math::Pose my_end;


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MovePerson)
}