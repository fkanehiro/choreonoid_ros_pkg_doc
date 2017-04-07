/**
  @file
  @author
 */

#include "BodyRosItem.h"
#include "BodyRosTorqueControllerItem.h"
#include "BodyRosHighgainControllerItem.h"
#include "WorldRosItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class RosBodyPlugin : public Plugin
{
public:
  RosBodyPlugin() : Plugin("RosBody") { }
  
  virtual bool initialize() {
    BodyRosItem::initialize(this);
    BodyRosTorqueControllerItem::initialize(this);
    BodyRosHighgainControllerItem::initialize(this);
    WorldRosItem::initialize(this);
    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(RosBodyPlugin);
