#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>


namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  //ModelPluginクラスを継承
  class VelodynePlugin : public ModelPlugin
  {

    private: 
      event::ConnectionPtr update_conn_;
      physics::ModelPtr model;
    /// \brief Pointer to the joint.
      physics::JointPtr joint;
      physics::WorldPtr world;
    /// \brief A PID controller for the joint.
      common::PID pid;
      common::Time sim_time;
      common::Time real_time;

    /// \brief Constructor
    public: VelodynePlugin() {}
    public: ~VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    //public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe velodyne plugin is attach to model[" <<
        _model->GetName() << "]\n";
        // Safety check
       if (_model->GetJointCount() == 0)
       {
         std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
         return;
       }

      
       this->update_conn_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&VelodynePlugin::OnUpdate, this));

       // Store the model pointer for convenience.
       this->model = _model;
       //モデルがいるWorldを取得する
       this->world = _model->GetWorld();
       
      physics::LinkPtr link = this->model->GetLink("my_velodyne::velodyne::top");
      std::cerr << link << "\n";
       

       // Get the first joint. We are making an assumption about the model
       // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[0];

       //// Setup a P-controller, with a gain of 0.1.
       //this->pid = common::PID(0.1, 0, 0);

       //// Apply the P-controller to the joint.
       //this->model->GetJointController()->SetVelocityPID(
       //    this->joint->GetScopedName(), this->pid);

       //// Set the joint's target velocity. This target velocity is just
       //// for demonstration purposes.
       //this->model->GetJointController()->SetVelocityTarget(
       //    this->joint->GetScopedName(), 10.0);

      //this->joint->SetParam("friction", 0, 0.0001);
      //this->joint->SetForce(2, 1);
      //ignition::math::Vector3d point1(0,0,0.1);
      //std::cerr << point1 << "\n";
      //link->SetTorque(point1);

    }

    public: void OnUpdate(){
      //std::cerr << "aaaa\n";
      this->joint->SetForce(0, 0.00001);
      this->sim_time = this->world->SimTime();
      this->real_time = this->world->RealTime();
      std::cerr << "smi time : " << this->sim_time << ",real time : "  << this->real_time << "\n";
      //ジョイントにトルクを発生させる
      //forceになっているのは他のジョイントの場合と共通化させるため
      //第一引数にはjointのインデックスを示す => 単一ジョイントの場合0のみだが2自由度ジョイントの場合は0 or 1となる
      //ignition::math::Vector3d point1(0,0,0.1);
      //link->SetTorque(point1);
    }

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
