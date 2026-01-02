#include "tortoisebot_gazebo_plugins/BallController.hpp"

#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"

#include "gz/sim/components/Pose.hh"

#include <gz/plugin/Register.hh>

using namespace gz;
using namespace sim;
using namespace tortoisebot;

class tortoisebot::BallControllerPrivate
{
    public:
        bool initialized = false;      // whether the controller has been initialized

        float mass = 1.0f;              // mass of the ball

        float speed = 1.0f;              // meters per second
        float radius = 3.0f;             // radius of the circular path

        float angularVelocity = speed / radius; // radians per second

        double timeElapsed = 0.0;        // time elapsed since start

        Model model{kNullEntity};        // The model this controller is attached to
        Link canonicalLink{kNullEntity}; // The canonical link of the model

        gz::math::Vector3d lastPose = {0.0f, 0.0f, 0.0f};
        gz::math::Vector3d lastVelocity = gz::math::Vector3d::Zero;
};

BallController::BallController(/* args */) : dataPtr(std::make_unique<BallControllerPrivate>())
{
};

BallController::~BallController()
{
};

void BallController::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
    this->dataPtr->model = Model(_entity);
    gzmsg << "tortoisebot::BallController::Configure for model ["
            << this->dataPtr->model.Name(_ecm) << "]" << std::endl;

    this->dataPtr->canonicalLink = Link(
        this->dataPtr->model.CanonicalLink(_ecm));
    gzmsg << "Canonical link is ["
            << this->dataPtr->canonicalLink.Name(_ecm).value_or("unknown") << "]"
            << std::endl;
    
    if (!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << "plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
        return;
    }

    // Read mass from SDF
    auto massElem = _sdf->FindElement("mass");
    if (massElem)
    {
        this->dataPtr->mass = massElem->Get<float>();
        gzmsg << "Mass set to [" << this->dataPtr->mass << "]" << std::endl;
    }

    // Read speed from SDF
    auto speedElem = _sdf->FindElement("ball_speed");
    if (speedElem)
    {
        this->dataPtr->speed = speedElem->Get<float>();
        gzmsg << "Speed set to [" << this->dataPtr->speed << "]" << std::endl;
    }

    // Read radius from SDF
    auto radiusElem = _sdf->FindElement("circle_radius");
    if (radiusElem)
    {
        this->dataPtr->radius = radiusElem->Get<float>();
        gzmsg << "Radius set to [" << this->dataPtr->radius << "]" << std::endl;
    }

    this->dataPtr->timeElapsed = 0.0;
};

void BallController::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    if (_info.paused) return;
    
    double dt = std::chrono::duration<double>(_info.dt).count();
    this->dataPtr->timeElapsed += dt;
    
    // gzmsg << "tortoisebot::BallController::PreUpdate for model ["
    //         << this->dataPtr->model.Name(_ecm) << "], dt [" << dt << "]"
    //         << std::endl;
    if (this->dataPtr->speed == 0.0f)
    {
        // No movement required
        return;
    }
    
    auto poseComp =
        _ecm.Component<gz::sim::components::Pose>(
        this->dataPtr->model.Entity());

    if (!poseComp) return;

    if (!this->dataPtr->initialized)
    {
        const auto &pos = poseComp->Data().Pos();

        // Initialize state FROM simulation, not from math
        this->dataPtr->lastPose = {pos.X(), pos.Y(), pos.Z()};
        this->dataPtr->lastVelocity = {0, 0, 0};
        this->dataPtr->timeElapsed = 0.0;

        this->dataPtr->initialized = true;

        gzmsg << "BallController initialized at pose "
            << pos << std::endl;
        return;
    }

    float angle = this->dataPtr->angularVelocity * this->dataPtr->timeElapsed;
    
    float x = this->dataPtr->radius * std::cos(angle);
    float y = this->dataPtr->radius * std::sin(angle);

    // gzmsg << "Pose: [" << poseComp->Data().Pos() << "], Expected Pose: ["
    // << this->dataPtr->lastPose[0] << ", " << this->dataPtr->lastPose[1] << ", " << this->dataPtr->lastPose[2] << "]" << std::endl;

    gz::math::Vector3d offsetVel = {
        (this->dataPtr->lastPose[0] - poseComp->Data().Pos().X()) / dt,
        (this->dataPtr->lastPose[1] - poseComp->Data().Pos().Y()) / dt,
        0.0f
    };
    
    this->dataPtr->lastPose = {x, y, 0.0f};

    gz::math::Vector3d desiredVel = {
        -y * this->dataPtr->speed,
        x * this->dataPtr->speed,
        0.0f        
    };

    desiredVel += offsetVel;
    // gzmsg << "Desired vel: [" << desiredVel << "], Current vel: [" << vel << "]" << std::endl;  

    gz::math::Vector3d accn = (desiredVel - this->dataPtr->lastVelocity) / dt;

    // gzmsg << "Acceleration: [" << accn << "]" << std::endl;

    gz::math::Vector3d force = accn * this->dataPtr->mass;

    // gzmsg << "Current Velocity: [" << vel 
    //         << "], Expected Velocity: [" << this->dataPtr->lastVelocity << "]" << std::endl;

    this->dataPtr->lastVelocity = desiredVel;

    // gzmsg << "Applying force [" << force << "] to link ["
    //         << this->dataPtr->canonicalLink.Name(_ecm).value_or("unknown") << "]"
    //         << std::endl;
    this->dataPtr->canonicalLink.AddWorldForce(_ecm, force);
};


GZ_ADD_PLUGIN(
    tortoisebot::BallController,
    gz::sim::System,
    tortoisebot::BallController::ISystemConfigure,
    tortoisebot::BallController::ISystemPreUpdate)