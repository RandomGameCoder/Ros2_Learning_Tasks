#ifndef TORTOISEBOT_GAZEBO_PLUGINS_BALL_CONTROLLER_HPP_
#define TORTOISEBOT_GAZEBO_PLUGINS_BALL_CONTROLLER_HPP_

#include <gz/sim/System.hh>
#include <memory>

namespace tortoisebot
{
    class BallControllerPrivate;
    class BallController : 
        public gz::sim::System, 
        public gz::sim::ISystemConfigure, 
        public gz::sim::ISystemPreUpdate
    {
    private:
        std::unique_ptr<BallControllerPrivate> dataPtr;
    public:
        BallController(/* args */);
        ~BallController();

        void Configure(
            const gz::sim::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &_sdf,
            gz::sim::EntityComponentManager &_ecm,
            gz::sim::EventManager &_eventMgr) override;
        void PreUpdate(
            const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;
    };
}

#endif  // TORTOISEBOT_GAZEBO_PLUGINS_BALL_CONTROLLER_HPP_