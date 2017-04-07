/**
   @file WorldRosItem.cpp
   @author
 */

#include "WorldRosItem.h"
#include "BodyRosItem.h"

#include <cnoid/BodyItem>
#include <cnoid/RootItem>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/EigenUtil>

using namespace cnoid;

#define DEBUG_WORLDROSITEM 0

/*
  Namepsace of parent of topics and services.
  NOTE: If you renaming, do not include '-'.
 */
const static std::string cnoidrospkg_parent_namespace_ = "choreonoid";

void WorldRosItem::initialize(ExtensionManager* ext)
{
  static bool initialized = false;
  int argc = 0;
  char** argv;
  if (!ros::isInitialized())
    ros::init(argc, argv, "choreonoid");
  if (!initialized) {
    ext->itemManager().registerClass<WorldRosItem>("WorldRosItem");
    ext->itemManager().addCreationPanel<WorldRosItem>();
    initialized = true;
  }
}

WorldRosItem::WorldRosItem()
{
  publish_clk_update_rate_ = 100.0;
  publish_ls_update_rate_  = 100.0;
  publish_ms_update_rate_  = 100.0;
  publish_cs_update_rate   = 100.0;
  is_csmsg_verbose         = false;

  registration_node_management_.clear();
  post_dynamics_function_regid.clear();

  RootItem::instance()->sigTreeChanged().connect(boost::bind(&WorldRosItem::registrationNodeStartAndStop, this));
}

WorldRosItem::WorldRosItem(const WorldRosItem& org)
  : Item(org)
{
  publish_clk_update_rate_ = org.publish_clk_update_rate_;
  publish_ls_update_rate_  = org.publish_ls_update_rate_;
  publish_ms_update_rate_  = org.publish_ms_update_rate_;
  publish_cs_update_rate   = org.publish_cs_update_rate;
  is_csmsg_verbose         = org.is_csmsg_verbose;

  registration_node_management_.clear();
  post_dynamics_function_regid.clear();

  RootItem::instance()->sigTreeChanged().connect(boost::bind(&WorldRosItem::registrationNodeStartAndStop, this));
}

WorldRosItem::~WorldRosItem()
{
  stop();
}

bool WorldRosItem::store(Archive& archive)
{
  archive.write("publishClockUpdateRate", publish_clk_update_rate_);
  archive.write("publishLinkStatesUpdateRate", publish_ls_update_rate_);
  archive.write("publishModelStatesUpdateRate", publish_ms_update_rate_);
  archive.write("publishContactsStateUpdateRate", publish_cs_update_rate);
  archive.write("contactsStateMessagesVerbose", is_csmsg_verbose);

  return true;
}

bool WorldRosItem::restore(const Archive& archive)
{
  archive.read("publishClockUpdateRate", publish_clk_update_rate_);
  archive.read("publishLinkStatesUpdateRate", publish_ls_update_rate_);
  archive.read("publishModelStatesUpdateRate", publish_ms_update_rate_);
  archive.read("publishContactsStateUpdateRate", publish_cs_update_rate);
  archive.read("contactsStateMessagesVerbose", is_csmsg_verbose);

  return true;
}

Item* WorldRosItem::doDuplicate() const
{
  return new WorldRosItem(*this);
}

void WorldRosItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty.decimals(2).min(0.0)("Publish clock update rate", publish_clk_update_rate_,
                                    changeProperty(publish_clk_update_rate_));
  putProperty.decimals(2).min(0.0)("Publish link states update rate", publish_ls_update_rate_,
                                    changeProperty(publish_ls_update_rate_));
  putProperty.decimals(2).min(0.0)("Publish model states update rate", publish_ms_update_rate_,
                                    changeProperty(publish_ms_update_rate_));
  putProperty.decimals(2).min(0.0)("Publish contacts state update rate", publish_cs_update_rate,
                                    changeProperty(publish_cs_update_rate));
  putProperty("Make contacts state messages verbose", is_csmsg_verbose, changeProperty(is_csmsg_verbose));

  return;
}

void WorldRosItem::publishContactsState()
{
  CollisionLinkPairListPtr collision_pairs;
  size_t                   i;

  if (! sim_access_ || ! sim_access_->get_collisions()) {
    return;
  }

  collision_pairs = sim_access_->get_collisions();
  i               = 0;

  for (CollisionLinkPairList::iterator it = collision_pairs->begin(); it != collision_pairs->end(); it++) {
    gazebo_msgs::ContactState dst;
    CollisionLinkPairPtr      p       = *it;
    size_t                    cols_sz = p->collisions.size();

    dst.info = "world:\"" + world->name() + "\"";

    /*
      Copy results of link pairs name.
     */
    dst.collision1_name = p->body[0]->name() + "::" + p->link[0]->name() + "::collision";
    dst.collision2_name = p->body[1]->name() + "::" + p->link[1]->name() + "::collision";

    /*
      Copy results of force and torque.
     */
    DyLink* dylink;
    size_t  wrch_sz;

    dylink  = dynamic_cast<DyLink*>(p->link[0]);
    wrch_sz = 0;

    if (dylink) {
      wrch_sz += dylink->constraintForces().size();
    }

    dst.wrenches.resize(wrch_sz);

    if (wrch_sz > 0) {
      Vector3 total_force(Vector3::Zero());
      Vector3 total_torque(Vector3::Zero());
      size_t  wrch_idx = 0;

      if (dylink) {
        Matrix3                       frame_rrot(Matrix3(dylink->R().inverse()));
        DyLink::ConstraintForceArray& cfa = dylink->constraintForces();

        for (size_t cfa_idx = 0; cfa_idx < cfa.size(); cfa_idx++) {
          Vector3 force = frame_rrot * cfa[cfa_idx].force;
          Vector3 tau   = frame_rrot *
                            (cfa[cfa_idx].point.cross(cfa[cfa_idx].force) - dylink->wc().cross(cfa[cfa_idx].force));

          dst.wrenches[wrch_idx].force.x  = force[0];
          dst.wrenches[wrch_idx].force.y  = force[1];
          dst.wrenches[wrch_idx].force.z  = force[2];
          dst.wrenches[wrch_idx].torque.x = tau[0];
          dst.wrenches[wrch_idx].torque.y = tau[1];
          dst.wrenches[wrch_idx].torque.z = tau[2];

          wrch_idx++;
        }

        total_force  = frame_rrot * dylink->f_ext();
        total_torque = frame_rrot * (dylink->tau_ext() - dylink->wc().cross(dylink->f_ext()));
      }

      dst.total_wrench.force.x  = total_force[0];
      dst.total_wrench.force.y  = total_force[1];
      dst.total_wrench.force.z  = total_force[2];
      dst.total_wrench.torque.x = total_torque[0];
      dst.total_wrench.torque.y = total_torque[1];
      dst.total_wrench.torque.z = total_torque[2];
    }

    /*
      Copy results of position and normal and depth.
     */
    dst.contact_positions.resize(cols_sz);
    dst.contact_normals.resize(cols_sz);
    dst.depths.resize(cols_sz);

    for (size_t j = 0; j < cols_sz; j++) {
      Collision* src = &(p->collisions[j]);

      dst.contact_positions[j].x = src->point[0];
      dst.contact_positions[j].y = src->point[1];
      dst.contact_positions[j].z = src->point[2];
      dst.contact_normals[j].x   = -src->normal[0];
      dst.contact_normals[j].y   = -src->normal[1];
      dst.contact_normals[j].z   = -src->normal[2];
      dst.depths[j]              = src->depth;
    }

    if (is_csmsg_verbose) {
      dst.info += (" number of collision pairs:(" + std::to_string(++i) + "/" +
                   std::to_string(collision_pairs->size()) + ") my geom:\"" + dst.collision1_name +
                   "\" other geom:\"" + dst.collision2_name + "\" wrenches/contacts:(" + std::to_string(wrch_sz) +
                   "/" + std::to_string(cols_sz) + ")");
    }

    dst.info += (" time:" + std::to_string(sim->currentTime()));

    contacts_state.states.push_back(dst);
  }

  if (publish_cs_next_time <= sim->currentTime()) {
    rosgraph_msgs::Clock tm;

    tm.clock.fromSec(sim->currentTime());

    contacts_state.header.stamp.sec  = tm.clock.sec;
    contacts_state.header.stamp.nsec = tm.clock.nsec;

    pub_world_contacts_state_.publish(contacts_state);
    contacts_state.states.clear();
    publish_cs_next_time += publish_cs_update_interval;
  }

  return;
}

void WorldRosItem::registrationNodeStartAndStop()
{
  WorldItemPtr parent;
  std::map<SimulatorItemPtr, std::string>::iterator it;

  if (! (parent = this->findOwnerItem<WorldItem>())) {
    return;
  }

  for (Item* child = parent->childItem(); child; child = child->nextItem()) {
    SimulatorItemPtr p = dynamic_cast<SimulatorItem*>(child);

    if (p && ! p->isRunning()) {
      it = registration_node_management_.find(p);

      if (it == registration_node_management_.end()) {
        p->sigSimulationStarted().connect(std::bind(&WorldRosItem::start, this));
        p->sigSimulationFinished().connect(std::bind(&WorldRosItem::stop, this));

        registration_node_management_[p] = p->name();
#if (DEBUG_WORLDROSITEM > 0)
        std::cout << registration_node_management_[p] << ": regsitered" << std::endl;
      } else {
        std::cout << it->second << ": already registered" << std::endl;
#endif  /* (DEBUG_WORLDROSITEM > 0) */
      }
    }
  }

  return;
}

void WorldRosItem::start()
{
  if (! (world = this->findOwnerItem<WorldItem>())) {
    return;
  } else if (! (sim = SimulatorItem::findActiveSimulatorItemFor(this))) {
    return;
  }

  ROS_DEBUG("Found WorldItem: %s", world->name().c_str());
  ROS_DEBUG("Found SimulatorItem: %s", sim->name().c_str());

  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(cnoidrospkg_parent_namespace_));

  if (publish_clk_update_rate_ > 0.0 && publish_clk_update_rate_ <= 1000.0) {
    pub_clock_                   = rosnode_->advertise<rosgraph_msgs::Clock>("/clock", 10);
    publish_clk_update_interval_ = 1.0 / publish_clk_update_rate_;
    publish_clk_next_time_       = 0.0;

    post_dynamics_function_regid.push_back(
      sim->addPostDynamicsFunction(std::bind(&WorldRosItem::publishClock, this)));
  }

  if (publish_ls_update_rate_ > 0.0 && publish_ls_update_rate_ <= 1000.0) {
    pub_link_states_            = rosnode_->advertise<gazebo_msgs::LinkStates>("link_states", 10);
    publish_ls_update_interval_ = 1.0 / publish_ls_update_rate_;
    publish_ls_next_time_       = 0.0;

    post_dynamics_function_regid.push_back(
      sim->addPostDynamicsFunction(std::bind(&WorldRosItem::publishLinkStates, this)));
  }

  if (publish_ms_update_rate_ > 0.0 && publish_ms_update_rate_ <= 1000.0) {
    pub_model_states_           = rosnode_->advertise<gazebo_msgs::ModelStates>("model_states", 10);
    publish_ms_update_interval_ = 1.0 / publish_ms_update_rate_;
    publish_ms_next_time_       = 0.0;

    post_dynamics_function_regid.push_back(
      sim->addPostDynamicsFunction(std::bind(&WorldRosItem::publishModelStates, this)));
  }

  if (publish_cs_update_rate > 0.0 && publish_cs_update_rate <= 1000.0) {
    std::string topic_name = world->name() + "/physics/contacts";
    std::replace(topic_name.begin(), topic_name.end(), '-', '_');

    pub_world_contacts_state_  = rosnode_->advertise<gazebo_msgs::ContactsState>(topic_name, 10);
    sim_access_                = static_cast<WorldRosSimulatorItemAccessor*>(sim.get());
    publish_cs_update_interval = 1.0 / publish_cs_update_rate;
    publish_cs_next_time       = 0.0;

    post_dynamics_function_regid.push_back(
      sim->addPostDynamicsFunction(std::bind(&WorldRosItem::publishContactsState, this)));

    AISTSimulatorItem* aist_sim;

    if ((aist_sim = static_cast<AISTSimulatorItem*>(sim.get()))) {
      aist_sim->setConstraintForceOutputEnabled(true);
    }

    contacts_state.states.clear();
  }

  std::string pause_physics_service_name("pause_physics");
  ros::AdvertiseServiceOptions pause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          pause_physics_service_name,
                                                          boost::bind(&WorldRosItem::pausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &rosqueue_);
  pause_physics_service_ = rosnode_->advertiseService(pause_physics_aso);

  std::string unpause_physics_service_name("unpause_physics");
  ros::AdvertiseServiceOptions unpause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          unpause_physics_service_name,
                                                          boost::bind(&WorldRosItem::unpausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &rosqueue_);
  unpause_physics_service_ = rosnode_->advertiseService(unpause_physics_aso);

  std::string reset_simulation_service_name("reset_simulation");
  ros::AdvertiseServiceOptions reset_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_simulation_service_name,
                                                          boost::bind(&WorldRosItem::resetSimulation,this,_1,_2),
                                                          ros::VoidPtr(), &rosqueue_);
  reset_simulation_service_ = rosnode_->advertiseService(reset_simulation_aso);

  std::string spawn_vrml_model_service_name("spawn_vrml_model");
  ros::AdvertiseServiceOptions spawn_vrml_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_vrml_model_service_name,
                                                                  boost::bind(&WorldRosItem::spawnModel,this,_1,_2),
                                                                  ros::VoidPtr(), &rosqueue_);
  spawn_vrml_model_service_ = rosnode_->advertiseService(spawn_vrml_model_aso);

  std::string spawn_urdf_model_service_name("spawn_urdf_model");
  ros::AdvertiseServiceOptions spawn_urdf_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_urdf_model_service_name,
                                                                  boost::bind(&WorldRosItem::spawnModel,this,_1,_2),
                                                                  ros::VoidPtr(), &rosqueue_);
  spawn_urdf_model_service_ = rosnode_->advertiseService(spawn_urdf_model_aso);

  std::string spawn_sdf_model_service_name("spawn_sdf_model");
  ros::AdvertiseServiceOptions spawn_sdf_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_sdf_model_service_name,
                                                                  boost::bind(&WorldRosItem::spawnModel,this,_1,_2),
                                                                  ros::VoidPtr(), &rosqueue_);
  spawn_sdf_model_service_ = rosnode_->advertiseService(spawn_sdf_model_aso);

  std::string delete_model_service_name("delete_model");
  ros::AdvertiseServiceOptions delete_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteModel>(
                                                                   delete_model_service_name,
                                                                   boost::bind(&WorldRosItem::deleteModel,this,_1,_2),
                                                                   ros::VoidPtr(), &rosqueue_);
  delete_model_service_ = rosnode_->advertiseService(delete_aso);

  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  rosqueue_thread_.reset(new boost::thread(&WorldRosItem::queueThread, this));
}

void WorldRosItem::publishClock()
{
  if (publish_clk_next_time_ > sim->currentTime()) {
    return;
  }

  rosgraph_msgs::Clock ros_time_;

  ros_time_.clock.fromSec(sim->currentTime());
  pub_clock_.publish(ros_time_);
  publish_clk_next_time_ += publish_clk_update_interval_;

  return;
}

void WorldRosItem::publishLinkStates()
{
  if (publish_ls_next_time_ > sim->currentTime()) {
    return;
  }

  gazebo_msgs::LinkStates link_states;

  Item* item = world->childItem();
  while(item) {
    BodyItem* body = dynamic_cast<BodyItem*>(item);
    if (body) {
      for (int i = 0; i < body->body()->numLinks(); i++) {
        Link* link = body->body()->link(i);
        link_states.name.push_back(body->name() + "::" + link->name());
        Vector3 pos = link->translation();
        Quaternion rot = Quaternion(link->rotation());
        geometry_msgs::Pose pose;
        pose.position.x = pos(0);
        pose.position.y = pos(1);
        pose.position.z = pos(2);
        pose.orientation.w = rot.w();
        pose.orientation.x = rot.x();
        pose.orientation.y = rot.y();
        pose.orientation.z = rot.z();
        link_states.pose.push_back(pose);
        /*
        twist.linear.x = linear_vel.x;
        twist.linear.y = linear_vel.y;
        twist.linear.z = linear_vel.z;
        twist.angular.x = angular_vel.x;
        twist.angular.y = angular_vel.y;
        twist.angular.z = angular_vel.z;
        link_states.twist.push_back(twist);
        */
      }
    }
    item = item->nextItem();
  }
  pub_link_states_.publish(link_states);
  publish_ls_next_time_ += publish_ls_update_interval_;

  return;
}

void WorldRosItem::publishModelStates()
{
  if (publish_ms_next_time_ > sim->currentTime()) {
    return;
  }

  gazebo_msgs::ModelStates model_states;

  Item* item = world->childItem();
  while(item) {
    BodyItem* body = dynamic_cast<BodyItem*>(item);
    if (body) {
      model_states.name.push_back(body->name());
      Link* link = body->body()->rootLink();
      Vector3 pos = link->translation();
      Quaternion rot = Quaternion(link->rotation());
      geometry_msgs::Pose pose;
      pose.position.x = pos(0);
      pose.position.y = pos(1);
      pose.position.z = pos(2);
      pose.orientation.w = rot.w();
      pose.orientation.x = rot.x();
      pose.orientation.y = rot.y();
      pose.orientation.z = rot.z();
      model_states.pose.push_back(pose);
      /*
      twist.linear.x = linear_vel.x;
      twist.linear.y = linear_vel.y;
      twist.linear.z = linear_vel.z;
      twist.angular.x = angular_vel.x;
      twist.angular.y = angular_vel.y;
      twist.angular.z = angular_vel.z;
      model_states.twist.push_back(twist);
      */
    }
    item = item->nextItem();
  }
  pub_model_states_.publish(model_states);
  publish_ms_next_time_ += publish_ms_update_interval_;

  return;
}

void WorldRosItem::queueThread()
{
  static const double timeout = 0.001;
  while (rosnode_->ok()) {
    rosqueue_.callAvailable(ros::WallDuration(timeout));
  }
}

bool WorldRosItem::pausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->pauseSimulation();
  return true;
}

bool WorldRosItem::unpausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->restartSimulation();
  return true;
}

bool WorldRosItem::resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->startSimulation(true);
  return true;
}

bool WorldRosItem::spawnModel(gazebo_msgs::SpawnModel::Request &req,
                              gazebo_msgs::SpawnModel::Response &res)
{
  std::string model_name = req.model_name;
  std::string model_xml = req.model_xml;

  cnoid::Vector3 trans;
  cnoid::Quaternion R;
  trans(0) = req.initial_pose.position.x;
  trans(1) = req.initial_pose.position.y;
  trans(2) = req.initial_pose.position.z;
  R.w() = req.initial_pose.orientation.w;
  R.x() = req.initial_pose.orientation.x;
  R.y() = req.initial_pose.orientation.y;
  R.z() = req.initial_pose.orientation.z;

  BodyItemPtr body = new BodyItem();
  body->setName(req.model_name);

  /*
    Load model file.
   */

  char tmpfname[L_tmpnam];
  bool is_loaded = false;

  memset(tmpfname, 0x00, sizeof(tmpfname));
  strncpy(tmpfname, "cnoid_ros_pkgXXXXXX", sizeof(tmpfname));

  if (mkstemp(tmpfname) != -1) {
    std::ofstream ofs(tmpfname);
    ofs << model_xml << std::endl;
    ofs.flush();
    ofs.close();
    is_loaded = body->loadModelFile(tmpfname);
    remove(tmpfname);
  }

  if (! is_loaded) {
    return false;
  }

  /*
    Spawn model to simulation world.
   */

  body->body()->rootLink()->setTranslation(trans);
  body->body()->rootLink()->setRotation(R.matrix());
  world->addChildItem(body);

  BodyRosItemPtr bodyros = new BodyRosItem();
  bodyros->setName(req.model_name + "_ROS");
  body->addChildItem(bodyros);
  
  ItemTreeView::instance()->checkItem(body);
  ItemTreeView::instance()->checkItem(bodyros);
  
  return true;
}

bool WorldRosItem::deleteModel(gazebo_msgs::DeleteModel::Request &req,
                               gazebo_msgs::DeleteModel::Response &res)
{
  ItemPtr item = world->findItem(req.model_name);
  if (!item)
  {
    ROS_ERROR("DeleteModel: model [%s] does not exist", req.model_name.c_str());
    res.success = false;
    res.status_message = "DeleteModel: model does not exist";
    return true;
  }
  item->detachFromParentItem();
  return true;
}

void WorldRosItem::stop()
{
  while (! post_dynamics_function_regid.empty()) {
    sim->removePostDynamicsFunction(post_dynamics_function_regid.back());
    post_dynamics_function_regid.pop_back();
  }

  if (ros::ok()) {
    if (rosqueue_.isEnabled()) {
      rosqueue_.clear();
    }

    if (async_ros_spin_) {
      async_ros_spin_->stop();
    }

    if (rosnode_) {
      rosnode_->shutdown();
    }
  }

  if (rosqueue_thread_) {
    rosqueue_thread_->join();
  }

  registration_node_management_.clear();

  return;
}
