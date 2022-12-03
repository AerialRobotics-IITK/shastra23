#include "shastra_magnet_plugin/dipole_magnet.hpp"

namespace gazebo
{

    DipoleMagnet::DipoleMagnet()
        : ModelPlugin()
    {
        this->connect_count_ = 0;
        gzmsg << "connection_count : " << this->connect_count_ << std::endl;
        gzmsg << "Dipole Moment plugin has been setup . Now feel the magnetism" << std::endl;
    }

    DipoleMagnet::~DipoleMagnet()
    {
        this->update_connection_.reset();
        if (this->should_publish_)
        {
            this->queue_.clear();
            this->queue_.disable();
            this->rosnode_->shutdown();
            this->callback_queue_thread_.join();
            delete this->rosnode_;
        }
        if (this->mag)
        {
            DipoleMagnetContainer::Get().Remove(this->mag);
        }
    }

    void DipoleMagnet::Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_)
    {
        // Store the pointer to the model
        this->model_ = parent_;
        this->world_ = parent_->GetWorld();
        gzdbg << "Loading DipoleMagnet plugin" << std::endl;

        this->mag = std::make_shared<DipoleMagnetContainer::Magnet>();

        // load parameters
        this->robot_namespace_ = "";
        if (sdf_->HasElement("robotNamespace"))
            this->robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>() + "/";

        if (!sdf_->HasElement("bodyName"))
        {
            gzerr << "DipoleMagnet plugin missing <bodyName>, cannot proceed fordward. Please, fix this !!" << std::endl;
            gzerr << "DipoleMagnet plugin exited due to above error !!" << std::endl;
            return;
        }
        else
        {
            this->link_name_ = sdf_->GetElement("bodyName")->Get<std::string>();
            gzmsg << "body_name " << sdf_->GetElement("bodyName")->Get<std::string>() << std::endl;
            gzmsg << sdf_->HasElement("dipole_moment") << std::endl;
            gzmsg << "dipole moment :" << sdf_->GetElement("dipole_moment")->Get<ignition::math::Vector3d>() << std::endl;
        }
        this->link_ = this->model_->GetLink(this->link_name_);
        if (!this->link_)
        {
            gzerr << "Error: link named " << this->link_name_ << " does not exist. Please, fix this to proceed fordward !!" << std::endl;
            gzerr << "DipoleMagnet plugin exited due to above error !!" << std::endl;
            return;
        }
        this->should_publish_ = false;
        if (sdf_->HasElement("shouldPublish"))
        {
            this->should_publish_ = sdf_->GetElement("shouldPublish")->Get<bool>();
        }

        if (!sdf_->HasElement("updateRate"))
        {
            gzmsg << "DipoleMagnet plugin missing <updateRate>, defaults to 0.0"
                     " (as fast as possible)"
                  << std::endl;
            this->update_rate_ = 0;
        }
        else
            this->update_rate_ = sdf_->GetElement("updateRate")->Get<double>();

        if (sdf_->HasElement("calculate"))
        {
            this->mag->calculate_ = sdf_->Get<bool>("calculate");
        }
        else
            this->mag->calculate_ = true;

        if (sdf_->HasElement("dipole_moment"))
        {
            this->mag->moment_ = sdf_->Get<ignition::math::Vector3d>("dipole_moment");
            gzmsg << "dipole moment :" << sdf_->GetElement("dipole_moment")->Get<ignition::math::Vector3d>() << std::endl;
        }
        else
            std::cout << "please provide dipole moment values" << std::endl;

        if (sdf_->HasElement("xyzOffset"))
        {
            this->mag->offset_.Pos() = sdf_->Get<ignition::math::Vector3d>("xyzOffset");
            gzmsg << "offset value : " << sdf_->GetElement("xyzOffset")->Get<ignition::math::Vector3d>() << std::endl;
        }

        if (sdf_->HasElement("rpyOffset"))
        {
            ignition::math::Vector3d rpy_offset = sdf_->Get<ignition::math::Vector3d>("rpyOffset");
            this->mag->offset_.Rot() = ignition::math::Quaternion<double>(rpy_offset);
        }

        if (sdf_->HasElement("Id"))
        {
            this->mag->model_id_ = sdf_->Get<int>("Id");
        }

        else
        {
            this->mag->model_id_ = this->model_->GetId();
        }

        if (this->should_publish_)
        {
            if (!sdf_->HasElement("topicNs"))
            {
                gzmsg << "DipoleMagnet plugin missing <topicNs>,"
                         "will publish on namespace "
                      << this->link_name_ << std::endl;
            }
            else
            {
                this->topic_ns = sdf_->GetElement("topicNs")->Get<std::string>();
            }

            if (!ros::isInitialized())
            {
                gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
                         "plugin. If you want to use this plugin without ROS, "
                         "set <shouldPublish> to false"
                      << std::endl;
                return;
            }

            this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
            this->rosnode_->setCallbackQueue(&this->queue_);

            this->wrench_pub_ = this->rosnode_->advertise<geometry_msgs::WrenchStamped>(this->topic_ns + "/wrench",
                                                                                        1,
                                                                                        boost::bind(&DipoleMagnet::Connect, this),
                                                                                        boost::bind(&DipoleMagnet::Disconnect, this),
                                                                                        ros::VoidPtr(),
                                                                                        &this->queue_);
            this->mfs_pub_ = this->rosnode_->advertise<sensor_msgs::MagneticField>(
                this->topic_ns + "/mfs", 1, boost::bind(&DipoleMagnet::Connect, this), boost::bind(&DipoleMagnet::Disconnect, this), ros::VoidPtr(), &this->queue_);

            // Custom Callback Queue
            this->callback_queue_thread_ = boost::thread(boost::bind(&DipoleMagnet::QueueThread, this));
        }

        gzmsg << "Loaded Gazebo dipole magnet plugin on " << this->model_->GetName() << std::endl;

        DipoleMagnetContainer::Get().Add(this->mag);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DipoleMagnet::OnUpdate, this, _1));
    }

    void DipoleMagnet::Connect()
    {
        this->connect_count_++;
    }

    void DipoleMagnet::Disconnect()
    {
        this->connect_count_--;
    }

    void DipoleMagnet::QueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosnode_->ok())
        {
            this->queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    // Called by the world update start event
    void DipoleMagnet::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Calculate the force from all other magnets
        ignition::math::Pose3d p_self = this->link_->WorldPose();
        p_self.Pos() += -p_self.Rot().RotateVector(this->mag->offset_.Pos());
        p_self.Rot() *= this->mag->offset_.Rot().Inverse();

        this->mag->pose_ = p_self;

        if (!this->mag->calculate_)
            return;

        DipoleMagnetContainer &dp = DipoleMagnetContainer::Get();

        ignition::math::Vector3d moment_world = p_self.Rot().RotateVector(this->mag->moment_);

        ignition::math::Vector3d force(0, 0, 0);
        ignition::math::Vector3d torque(0, 0, 0);
        ignition::math::Vector3d mfs(0, 0, 0);
        for (DipoleMagnetContainer::MagnetPtrV::iterator it = dp.magnets_.begin(); it < dp.magnets_.end(); it++)
        {
            std::shared_ptr<DipoleMagnetContainer::Magnet> mag_other = *it;
            if (mag_other->model_id_ != this->mag->model_id_)
            {
                ignition::math::Pose3d p_other = mag_other->pose_;
                ignition::math::Vector3d m_other = p_other.Rot().RotateVector(mag_other->moment_);

                ignition::math::Vector3d force_tmp;
                ignition::math::Vector3d torque_tmp;
                getForceTorque(p_self, moment_world, p_other, m_other, force_tmp, torque_tmp);

                force += force_tmp;
                torque += torque_tmp;

                ignition::math::Vector3d mfs_tmp;
                getMFS(p_self, p_other, m_other, mfs_tmp);

                mfs += mfs_tmp;

                this->link_->AddForce(force_tmp);
                this->link_->AddTorque(torque_tmp);
            }
        }

        this->publishData(force, torque, mfs);
    }

    void DipoleMagnet::publishData(const ignition::math::Vector3d &force, const ignition::math::Vector3d &torque, const ignition::math::Vector3d &mfs)
    {
        if (this->should_publish_ && this->connect_count_ > 0)
        {
            // Rate control
            common::Time cur_time = this->world_->SimTime();
            if (this->update_rate_ > 0 && (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
                return;

            this->lock_.lock();
            // copy data into wrench message
            this->wrench_msg_.header.frame_id = "world_";
            this->wrench_msg_.header.stamp.sec = cur_time.sec;
            this->wrench_msg_.header.stamp.nsec = cur_time.nsec;

            this->wrench_msg_.wrench.force.x = force.X();
            this->wrench_msg_.wrench.force.y = force.Y();
            this->wrench_msg_.wrench.force.z = force.Z();
            this->wrench_msg_.wrench.torque.x = torque.X();
            this->wrench_msg_.wrench.torque.y = torque.Y();
            this->wrench_msg_.wrench.torque.z = torque.Z();

            // now mfs
            this->mfs_msg_.header.frame_id = this->link_name_;
            this->mfs_msg_.header.stamp.sec = cur_time.sec;
            this->mfs_msg_.header.stamp.nsec = cur_time.nsec;

            this->mfs_msg_.magnetic_field.x = mfs.X();
            this->mfs_msg_.magnetic_field.y = mfs.Y();
            this->mfs_msg_.magnetic_field.z = mfs.Z();

            this->wrench_pub_.publish(this->wrench_msg_);
            this->mfs_pub_.publish(this->mfs_msg_);

            this->lock_.unlock();
        }
    }

    void DipoleMagnet::getForceTorque(const ignition::math::Pose3d &p_self,
                                      const ignition::math::Vector3d &m_self,
                                      const ignition::math::Pose3d &p_other,
                                      const ignition::math::Vector3d &m_other,
                                      ignition::math::Vector3d &force,
                                      ignition::math::Vector3d &torque)
    {
        bool debug = false;
        ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
        ignition::math::Vector3d p_unit = p / p.Length();

        ignition::math::Vector3d m1 = m_other;
        ignition::math::Vector3d m2 = m_self;
        if (debug)
        {
            gzmsg << "p: " << p << " m1: " << m1 << " m2: " << m2 << std::endl;
        }

        double K = 3.0 * 1e-7 / pow(p.Length(), 4);
        force = K * (m2 * (m1.Dot(p_unit)) + m1 * (m2.Dot(p_unit)) + p_unit * (m1.Dot(m2)) - 5 * p_unit * (m1.Dot(p_unit)) * (m2.Dot(p_unit)));

        double Ktorque = 1e-7 / pow(p.Length(), 3);
        ignition::math::Vector3d B1 = Ktorque * (3 * (m1.Dot(p_unit)) * p_unit - m1);
        torque = m2.Cross(B1);
        if (debug)
        {
            gzmsg << "B: " << B1 << " K: " << Ktorque << " t: " << torque << std::endl;
        }
    }

    void DipoleMagnet::getMFS(const ignition::math::Pose3d &p_self,
                              const ignition::math::Pose3d &p_other,
                              const ignition::math::Vector3d &m_other,
                              ignition::math::Vector3d &mfs)
    {
        // sensor location
        ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
        ignition::math::Vector3d p_unit = p / p.Length();

        // Get the field at the sensor location
        double K = 1e-7 / pow(p.Length(), 3);
        ignition::math::Vector3d B = K * (3 * (m_other.Dot(p_unit)) * p_unit - m_other);

        // Rotate the B vector into the capsule/body frame
        ignition::math::Vector3d B_body = p_self.Rot().RotateVectorReverse(B);

        // Assign vector
        mfs.X() = B_body[0];
        mfs.Y() = B_body[1];
        mfs.Z() = B_body[2];
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DipoleMagnet)

} // namespace gazebo