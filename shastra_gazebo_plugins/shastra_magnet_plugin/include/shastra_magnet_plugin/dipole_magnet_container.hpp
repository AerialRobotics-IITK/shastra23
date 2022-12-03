#pragma once

#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace gazebo
{

    class DipoleMagnetContainer
    {
    public:
        DipoleMagnetContainer()
        {
        }

        static DipoleMagnetContainer &Get()
        {
            static DipoleMagnetContainer instance;
            return instance;
        }

        struct Magnet
        {
            bool calculate_;
            ignition::math::Vector3d moment_;
            ignition::math::Pose3d offset_;
            ignition::math::Pose3d pose_;
            std::uint32_t model_id_;
        };

        typedef std::shared_ptr<Magnet> MagnetPtr;
        typedef std::vector<MagnetPtr> MagnetPtrV;

        void Add(MagnetPtr mag)
        {
            gzmsg << "Adding mag id:" << mag->model_id_ << std::endl;
            this->magnets_.push_back(mag);
            gzmsg << "Total: " << this->magnets_.size() << " magnets" << std::endl;
        }
        void Remove(MagnetPtr mag)
        {
            gzmsg << "Removing mag id:" << mag->model_id_ << std::endl;
            this->magnets_.erase(std::remove(this->magnets_.begin(), this->magnets_.end(), mag), this->magnets_.end());
            gzmsg << "Total: " << this->magnets_.size() << " magnets" << std::endl;
        }

        MagnetPtrV magnets_;
    };

} // namespace gazebo