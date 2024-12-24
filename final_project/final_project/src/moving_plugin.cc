#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo {
class MovingPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;

    // Set params
    std::cout << "Load before params" << "\n";

    // linear_vel
    this->linear_vel = 0.1;
    if (_sdf->HasElement("linear_vel")) {
      this->linear_vel = _sdf->Get<double>("linear_vel");
    }
    std::cout << this->linear_vel << "\n";

    // iterations
    this->iterations = 10 * 1000;
    if (_sdf->HasElement("iterations")) {
      this->iterations = _sdf->Get<int>("iterations");
    }
    std::cout << this->iterations << "\n";

    // axis
    this->axis = 1;
    if (_sdf->HasElement("axis")) {
      this->axis = _sdf->Get<int>("axis");
    }
    std::cout << this->axis << "\n";

    // start_direction
    this->start_direction = 1;
    if (_sdf->HasElement("start_direction")) {
      this->start_direction = _sdf->Get<int>("start_direction");
    }
    std::cout << this->start_direction << "\n";

    std::cout << "Load after params" << "\n";

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MovingPlugin::OnUpdate, this));
  }

  // Called by the world update start event
public:
  void OnUpdate() {
    double vel = this->start_direction * this->linear_vel;

    if (this->counter > this->iterations) {
      vel = -vel;
    }

    switch (this->axis) {
      case 1:
        this->model->SetLinearVel(ignition::math::Vector3d(vel, 0, 0));
        break;
      case 2:
        this->model->SetLinearVel(ignition::math::Vector3d(0, vel, 0));
        break;
    }

    this->counter++;

    if (this->counter > (2 * this->iterations)) {
      this->counter = 0;
    }
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

private:
  int counter;
  double linear_vel;
  int iterations;
  int axis;
  int start_direction;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MovingPlugin)
} // namespace gazebo