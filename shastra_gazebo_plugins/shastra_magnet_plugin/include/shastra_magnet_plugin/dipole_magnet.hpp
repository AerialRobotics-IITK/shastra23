#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/MagneticField.h>

#include <ignition/math/Vector3.hh>
#include <memory>

#include "shastra_magnet_plugin/dipole_magnet_container.hpp"

namespace gazebo
{

    class DipoleMagnet : public ModelPlugin
    {
    public:
        DipoleMagnet();

        ~DipoleMagnet();

        /// \brief Loads the plugin
        void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_);

        /// \brief Callback for when subscribers connect
        void Connect();

        /// \brief Callback for when subscribers disconnect
        void Disconnect();

        /// \brief Thread to interact with ROS
        void QueueThread();

        /// \brief Called by the world update start event
        void OnUpdate(const common::UpdateInfo & /*_info*/);

        /// \brief Publishes data to ros topics
        /// \pram[in] force A vector of force that makes up the wrench to be published
        /// \pram[in] torque A vector of torque that makes up the wrench to be published
        /// \pram[in] mfs A vector of magnetic field data
        void publishData(const ignition::math::Vector3d &force, const ignition::math::Vector3d &torque, const ignition::math::Vector3d &mfs);

        /// \brief Calculate force and torque of a magnet on another
        /// \parama[in] p_self Pose of the first magnet
        /// \parama[in] m_self Dipole moment of the first magnet
        /// \parama[in] p_other Pose of the second magnet
        /// \parama[in] m_other Dipole moment of the second magnet on which the force is calculated
        /// \param[out] force Calculated force vector
        /// \param[out] torque Calculated torque vector
        void getForceTorque(const ignition::math::Pose3d &p_self,
                            const ignition::math::Vector3d &m_self,
                            const ignition::math::Pose3d &p_other,
                            const ignition::math::Vector3d &m_other,
                            ignition::math::Vector3d &force,
                            ignition::math::Vector3d &torque);

        /// \brief Calculate the magnetic field on all 6 sensors
        /// \parama[in] p_self Pose of the first magnet
        /// \parama[in] p_other Pose of the second magnet
        /// \parama[in] m_other Dipole moment of the second magnet
        /// \param[out] mfs magnetic field sensors
        void
        getMFS(const ignition::math::Pose3d &p_self, const ignition::math::Pose3d &p_other, const ignition::math::Vector3d &m_other, ignition::math::Vector3d &mfs);

        // Pointer to the model
    private:
        physics::ModelPtr model_;
        physics::LinkPtr link_;
        physics::WorldPtr world_;

        std::shared_ptr<DipoleMagnetContainer::Magnet> mag;

        std::string link_name_;
        std::string robot_namespace_;
        std::string topic_ns;

        bool should_publish_;
        ros::NodeHandle *rosnode_;
        ros::Publisher wrench_pub_;
        ros::Publisher mfs_pub_;

        geometry_msgs::WrenchStamped wrench_msg_;
        sensor_msgs::MagneticField mfs_msg_;

    private:
        boost::mutex lock_;
        int connect_count_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        common::Time last_time_;
        double update_rate_;
        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
    };

} // namespace gazebo