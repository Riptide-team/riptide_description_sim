#include <cstdlib>
// #include <ignition/transport/TopicUtils.hh>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <iostream>

#include <ignition/msgs/double.pb.h>

#include <ignition/math/Helpers.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/BatteryPowerLoad.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "Thruster.hh"

class riptide_plugins::ThrusterPrivateData {
  /// \brief The mode of operation
  public: 

  /// \brief Thrust output by propeller in N
  public: double thrust = 0.0;

  /// \brief Desired propeller angular velocity in rad / s
  public: double propellerAngVel = 0.0;

  /// \brief Enabled or not
  public: bool enabled = true;

  /// \brief Model entity
  public: gz::sim::Entity modelEntity;

  /// \brief The link entity which will spin
  public: gz::sim::Entity linkEntity;

  /// \brief Axis along which the propeller spins. Expressed in the joint
  /// frame. Addume this doesn't change during simulation.
  public: gz::math::Vector3d jointAxis;

  /// \brief Joint pose in the child link frame. Assume this doesn't change
  /// during the simulation.
  public: gz::math::Pose3d jointPose;

  /// \brief Propeller koint entity
  public: gz::sim::Entity jointEntity;

  /// \brief Thrust coefficient relating the propeller angular velocity to the
  /// thrust
  public: double thrustCoefficient = 1;

  /// \brief Function which computers thrust from angular velocity
  /// \param[in] _angVel Angular Velocity in rad/s
  /// \return Thrust in Newtons
  public: double AngularVelToThrust(double _angVel);
};

/////////////////////////////////////////////////
namespace riptide_plugins {
    Thruster::Thruster():
        dataPtr(std::make_unique<ThrusterPrivateData>()) {
    }

    /////////////////////////////////////////////////
    void Thruster::Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/)
    {
    // Create model object, to access convenient functions
    this->dataPtr->modelEntity = _entity;
    auto model = gz::sim::Model(_entity);
    auto modelName = model.Name(_ecm);

        // Get joint name
        if (!_sdf->HasElement("joint_name"))
        {
            std::cerr << "Missing <joint_name>. Plugin won't be initialized."
                << std::endl;
            return;
        }
        auto jointName = _sdf->Get<std::string>("joint_name");

        // Get thrust coefficient
        if (_sdf->HasElement("thrust_coefficient"))
        {
            this->dataPtr->thrustCoefficient = _sdf->Get<double>("thrust_coefficient");
        }

        this->dataPtr->jointEntity = model.JointByName(_ecm, jointName);
        if (gz::sim::kNullEntity == this->dataPtr->jointEntity) {
            std::cerr << "Failed to find joint [" << jointName << "] in model ["
                << modelName << "]. Plugin not initialized." << std::endl;
            return;
        }

        this->dataPtr->jointAxis =
            _ecm.Component<gz::sim::components::JointAxis>(
            this->dataPtr->jointEntity)->Data().Xyz();

        this->dataPtr->jointPose = _ecm.Component<gz::sim::components::Pose>(
            this->dataPtr->jointEntity)->Data();

        // Get link entity
        auto childLink =
            _ecm.Component<gz::sim::components::ChildLinkName>(
            this->dataPtr->jointEntity);
        this->dataPtr->linkEntity = model.LinkByName(_ecm, childLink->Data());

        // Create necessary components if not present.
        gz::sim::enableComponent<gz::sim::components::AngularVelocity>(_ecm, this->dataPtr->linkEntity);
        gz::sim::enableComponent<gz::sim::components::WorldAngularVelocity>(_ecm, this->dataPtr->linkEntity);
        gz::sim::enableComponent<gz::sim::components::WorldLinearVelocity>(_ecm, this->dataPtr->linkEntity);
    }

    /////////////////////////////////////////////////
    double ThrusterPrivateData::AngularVelToThrust(double _angVel) {
        // Thrust computed from the angular velocity
        return this->thrustCoefficient * abs(_angVel) * _angVel;
    }

    /////////////////////////////////////////////////
    void Thruster::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
        if (_info.paused)
            return;

        if (!this->dataPtr->enabled)
        {
            return;
        }
        if (!_ecm.HasEntity(this->dataPtr->linkEntity)){
            return;
        }

        gz::sim::Link link(this->dataPtr->linkEntity);

        auto pose = worldPose(this->dataPtr->linkEntity, _ecm);

        // TODO(arjo129): add logic for custom coordinate frame
        // Convert joint axis to the world frame
        const auto linkWorldPose = worldPose(this->dataPtr->linkEntity, _ecm);
        auto jointWorldPose = linkWorldPose * this->dataPtr->jointPose;
        auto unitVector = jointWorldPose.Rot().RotateVector(this->dataPtr->jointAxis).Normalize();

        // Getting angular velocity
        auto currentAngular = (link.WorldAngularVelocity(_ecm))->Dot(unitVector);

        // Computing the th
        double thrust = this->dataPtr->AngularVelToThrust(currentAngular);
        double torque = 0.;

        // Applying wrench
        link.AddWorldWrench( _ecm, unitVector * thrust, unitVector * torque);
    }

    // void Thruster::PostUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
        // Nothing to do
    // }
} // namespace riptide_plugins


IGNITION_ADD_PLUGIN(
  riptide_plugins::Thruster, ignition::gazebo::System,
  riptide_plugins::Thruster::ISystemConfigure,
  riptide_plugins::Thruster::ISystemPreUpdate
//   riptide_plugins::Thruster::ISystemPostUpdate
)