/**
   @file WorldRosItem.h
   @author
 */

#ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/DyBody>
#include <cnoid/WorldItem>
#include <cnoid/SimulatorItem>
#include <cnoid/AISTSimulatorItem>
#include <cnoid/TimeBar>
#include "exportdecl.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ContactsState.h>

#include <vector>
#include <boost/thread.hpp>

namespace cnoid {

/**
   @brief This class is for accessing protected 'getCollisions' method in the SimulatorItem class.
   The use of this class is limited to this only. How to use, please see following example.

   @code
   SimulatorItemPtr               p;
   WorldRosSimulatorItemAccessor* sim_access;

   p          = <set SimulatorItem's instance>;
   sim_access = static_cast<WorldRosSimulatorItemAccessor*>(p.get());

   CollisionsLinkPairListPtr link_pairs = sim_access->get_collisions();

   if (link_pairs) {
     // This physics engine are collision output supported.
   } else {
     // This physics engine are not collision output supported.
   }
   @endcode

   @attention This class does not consider usage other than the contents described in the explanation.
 */
class CNOID_EXPORT WorldRosSimulatorItemAccessor : public SimulatorItem
{
public:
  WorldRosSimulatorItemAccessor() { }
  CollisionLinkPairListPtr get_collisions() { return getCollisions(); }
  virtual SimulationBody* createSimulationBody(Body* orgBody) { return 0; }
  virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) { return true; }
  virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) { return true; }
};

typedef ref_ptr<WorldRosSimulatorItemAccessor> WorldRosSimulatorItemAccessorPtr;

/**
   @brief This class is provides all services and some topics (link state, model state, contact state).
 */
class CNOID_EXPORT WorldRosItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);
    
    WorldRosItem();
    WorldRosItem(const WorldRosItem& org);
    virtual ~WorldRosItem();

    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    void start();
    void stop();
    
    void setModuleName(const std::string& name);

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    
private:
    WorldItemPtr world;
    SimulatorItemPtr sim;
    boost::shared_ptr<ros::NodeHandle> rosnode_;
    boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
    ros::CallbackQueue rosqueue_;
    boost::shared_ptr<boost::thread> rosqueue_thread_;

    /**
       The variable for managing registration to the simulator item.
       This registration that function of node creation at the simulation start,
       node deletion at the simulation stop.
     */
    std::map<SimulatorItemPtr, std::string> registration_node_management_;

    /// The registration id of calling function from physics engine. (physics engine is SimulatorItem's subclass)
    std::list<int> post_dynamics_function_regid;

    void queueThread();

    /**
      @brief Connect to event signal, function of nodes creation and deletion.
      The event signal is 'RootItem::instace()->sigTreeChanged()'.
     */
    void registrationNodeStartAndStop();

    /*
      For services and topics.
     */

    /// Update rate for publish clock.
    double publish_clk_update_rate_;
    /// Update interval for publish clock.
    double publish_clk_update_interval_;
    /// Publish next time for clock.
    double publish_clk_next_time_;

    /// Update rate for publish link states.
    double publish_ls_update_rate_;
    /// Update interval for publish link states.
    double publish_ls_update_interval_;
    /// Publish next time for link states.
    double publish_ls_next_time_;

    /// Update rate for publish model states.
    double publish_ms_update_rate_;
    /// Update interval for publish model states.
    double publish_ms_update_interval_;
    /// Publish next time for model states.
    double publish_ms_next_time_;

    void publishClock();
    void publishLinkStates();
    void publishModelStates();
    bool resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool pausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool unpausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool spawnModel(gazebo_msgs::SpawnModel::Request &req, gazebo_msgs::SpawnModel::Response &res);
    bool deleteModel(gazebo_msgs::DeleteModel::Request &req, gazebo_msgs::DeleteModel::Response &res);

    ros::Publisher     pub_clock_;
    ros::Publisher     pub_link_states_;
    ros::Publisher     pub_model_states_;
    ros::ServiceServer reset_simulation_service_;
    ros::ServiceServer reset_world_service_;
    ros::ServiceServer pause_physics_service_;
    ros::ServiceServer unpause_physics_service_;
    ros::ServiceServer spawn_vrml_model_service_;
    ros::ServiceServer spawn_urdf_model_service_;
    ros::ServiceServer spawn_sdf_model_service_;
    ros::ServiceServer delete_model_service_;

    /*
      For publish collision data. (contacts state)
     */

    /// Publisher of world contacts state. (link collisions pair)
    ros::Publisher pub_world_contacts_state_;
    /// For getting collision data.
    WorldRosSimulatorItemAccessor* sim_access_;

    /// Update rate for publish contacts state.
    double publish_cs_update_rate;
    /// Update interval for publish contacts state.
    double publish_cs_update_interval;
    /// Publish next time for contacts state.
    double publish_cs_next_time;
    /// Choose contacts state messages quiet or verbose.
    bool is_csmsg_verbose;

    /// Publish this message.
    gazebo_msgs::ContactsState contacts_state;

    /**
       @brief Publish link conatcts state.
       This information is the calculation result in the physics engine. (e.g. AISTSimulatorItem etc)
     */
    void publishContactsState();
};

typedef ref_ptr<WorldRosItem> WorldRosItemPtr;

}
#endif /* #ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED */
