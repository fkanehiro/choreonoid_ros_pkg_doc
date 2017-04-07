#include "BodyRosItem.h"
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <ros/console.h>

using namespace cnoid;

void BodyRosItem::initialize(ExtensionManager* ext) { 
  static bool initialized = false;
  int argc = 0;
  char** argv;
  if (!ros::isInitialized())
    ros::init(argc, argv, "choreonoid");
  if (!initialized) {
    ext->itemManager().registerClass<BodyRosItem>("BodyRosItem");
    ext->itemManager().addCreationPanel<BodyRosItem>();
    initialized = true;
  }
}

BodyRosItem::BodyRosItem()
  : os(MessageView::instance()->cout())
{
  controllerTarget = NULL;
  joint_state_update_rate_ = 100.0;
}

BodyRosItem::BodyRosItem(const BodyRosItem& org)
  : ControllerItem(org),
    os(MessageView::instance()->cout())
{
  controllerTarget = NULL;
  joint_state_update_rate_ = 100.0;
}

BodyRosItem::~BodyRosItem()
{
  stop();
}

Item* BodyRosItem::doDuplicate() const
{
  return new BodyRosItem(*this);
}

bool BodyRosItem::store(Archive& archive)
{
  archive.write("jointStateUpdateRate", joint_state_update_rate_);

  return true;
}

bool BodyRosItem::restore(const Archive& archive)
{
  archive.read("jointStateUpdateRate", joint_state_update_rate_);

  return true;
}

void BodyRosItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty.decimals(2).min(0.0)("Update rate", joint_state_update_rate_, changeProperty(joint_state_update_rate_));
}

bool BodyRosItem::start(Target* target)
{
  if (! target) {
    MessageView::instance()->putln(MessageView::ERROR, boost::format("Target not found"));
    return false;
  } else if (! target->body()) {
    MessageView::instance()->putln(MessageView::ERROR, boost::format("BodyItem not found"));
    return false;
  }

  controllerTarget = target;
  simulationBody = target->body();
  timeStep_ = target->worldTimeStep();
  controlTime_ = target->currentTime();

  // buffer of preserve currently state of joints.
  joint_state_.header.stamp.fromSec(controlTime_);
  joint_state_.name.resize(body()->numJoints());
  joint_state_.position.resize(body()->numJoints());
  joint_state_.velocity.resize(body()->numJoints());
  joint_state_.effort.resize(body()->numJoints());

  // preserve initial state of joints.
  for (size_t i = 0; i < body()->numJoints(); i++) {
    Link* joint = body()->joint(i);

    joint_state_.name[i]     = joint->name();
    joint_state_.position[i] = joint->q();
    joint_state_.velocity[i] = joint->dq();
    joint_state_.effort[i]   = joint->u();
  }

  std::string name = simulationBody->name();
  std::replace(name.begin(), name.end(), '-', '_');
  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));
  createSensors(simulationBody);

  joint_state_publisher_     = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1000);
  joint_state_update_period_ = 1.0 / joint_state_update_rate_;
  joint_state_last_update_   = controllerTarget->currentTime();
  ROS_DEBUG("Joint state update rate %f", joint_state_update_rate_);

  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();
  
  return true;
}

bool BodyRosItem::createSensors(BodyPtr body)
{
  forceSensors_ = body->devices<ForceSensor>().getSortedById();
  gyroSensors_ = body->devices<RateGyroSensor>().getSortedById();
  accelSensors_ = body->devices<AccelerationSensor>().getSortedById();
  visionSensors_ = body->devices<Camera>().getSortedById();
  rangeVisionSensors_ = body->devices<RangeCamera>().getSortedById();
  rangeSensors_ = body->devices<RangeSensor>().getSortedById();

  force_sensor_publishers_.resize(forceSensors_.size());
  for (size_t i=0; i < forceSensors_.size(); ++i) {
    if (ForceSensor* sensor = forceSensors_[i]) {
      force_sensor_publishers_[i] = rosnode_->advertise<geometry_msgs::Wrench>(sensor->name(), 1);
      sensor->sigStateChanged().connect(boost::bind(&BodyRosItem::updateForceSensor,
                                                    this, sensor, force_sensor_publishers_[i]));
      ROS_DEBUG("Create force sensor %s with cycle %f", sensor->name().c_str(), sensor->cycle());
    }
  }
  rate_gyro_sensor_publishers_.resize(gyroSensors_.size());
  for (size_t i=0; i < gyroSensors_.size(); ++i) {
    if (RateGyroSensor* sensor = gyroSensors_[i]) {
      rate_gyro_sensor_publishers_[i] = rosnode_->advertise<sensor_msgs::Imu>(sensor->name(), 1);
      sensor->sigStateChanged().connect(boost::bind(&BodyRosItem::updateRateGyroSensor,
                                                    this, sensor, rate_gyro_sensor_publishers_[i]));
      ROS_DEBUG("Create gyro sensor %s with cycle %f", sensor->name().c_str(), sensor->cycle());
    }
  }
  accel_sensor_publishers_.resize(accelSensors_.size());
  for (size_t i=0; i < accelSensors_.size(); ++i) {
    if (AccelerationSensor* sensor = accelSensors_[i]) {
      accel_sensor_publishers_[i] = rosnode_->advertise<sensor_msgs::Imu>(sensor->name(), 1);
      sensor->sigStateChanged().connect(boost::bind(&BodyRosItem::updateAccelSensor,
                                                    this, sensor, accel_sensor_publishers_[i]));
      ROS_DEBUG("Create accel sensor %s with cycle %f", sensor->name().c_str(), sensor->cycle());
    }
  }
  image_transport::ImageTransport it(*rosnode_);
  vision_sensor_publishers_.resize(visionSensors_.size());
  for (size_t i=0; i < visionSensors_.size(); ++i) {
    if (Camera* sensor = visionSensors_[i]) {
      vision_sensor_publishers_[i] = it.advertise(sensor->name() + "/image_raw", 1);
      sensor->sigStateChanged().connect(boost::bind(&BodyRosItem::updateVisionSensor,
                                                    this, sensor, vision_sensor_publishers_[i]));
      ROS_DEBUG("Create vision sensor %s with cycle %f", sensor->name().c_str(), sensor->cycle());
    }
  }
  range_vision_sensor_publishers_.resize(rangeVisionSensors_.size());
  for (size_t i=0; i < rangeVisionSensors_.size(); ++i) {
    if (RangeCamera* sensor = rangeVisionSensors_[i]) {
      range_vision_sensor_publishers_[i] = rosnode_->advertise<sensor_msgs::PointCloud2>(sensor->name() + "/point_cloud", 1);
      sensor->sigStateChanged().connect(boost::bind(&BodyRosItem::updateRangeVisionSensor,
                                                    this, sensor, range_vision_sensor_publishers_[i]));
      ROS_DEBUG("Create RGBD sensor %s with cycle %f", sensor->name().c_str(), sensor->cycle());
    }
  }
  range_sensor_publishers_.resize(rangeSensors_.size());
  for (size_t i=0; i < rangeSensors_.size(); ++i) {
    if (RangeSensor* sensor = rangeSensors_[i]) {
      range_sensor_publishers_[i] = rosnode_->advertise<sensor_msgs::LaserScan>(sensor->name(), 1);
      sensor->sigStateChanged().connect(boost::bind(&BodyRosItem::updateRangeSensor,
                                                    this, sensor, range_sensor_publishers_[i]));
      ROS_DEBUG("Create range sensor %s with cycle %f", sensor->name().c_str(), sensor->cycle());
    }
  }
}

bool BodyRosItem::control()
{
  controlTime_ = controllerTarget->currentTime();
  double updateSince = controlTime_ - joint_state_last_update_;

  if (updateSince > joint_state_update_period_) {
    // publish current joint states
    joint_state_.header.stamp.fromSec(controlTime_);

    for (int i = 0; i < body()->numJoints(); i++) {
      Link* joint = body()->joint(i);

      joint_state_.position[i] = joint->q();
      joint_state_.velocity[i] = joint->dq();
      joint_state_.effort[i]   = joint->u();
    }

    joint_state_publisher_.publish(joint_state_);
    joint_state_last_update_ += joint_state_update_period_;
  }

  return true;
}

void BodyRosItem::updateForceSensor(ForceSensor* sensor, ros::Publisher& publisher)
{
  geometry_msgs::Wrench force;
  force.force.x = sensor->F()[0];
  force.force.y = sensor->F()[1];
  force.force.z = sensor->F()[2];
  force.torque.x = sensor->F()[3];
  force.torque.y = sensor->F()[4];
  force.torque.z = sensor->F()[5];
  publisher.publish(force);
}

void BodyRosItem::updateRateGyroSensor(RateGyroSensor* sensor, ros::Publisher& publisher)
{
  sensor_msgs::Imu gyro;
  gyro.header.stamp.fromSec(controllerTarget->currentTime());
  gyro.header.frame_id = sensor->name();
  gyro.angular_velocity.x = sensor->w()[0];
  gyro.angular_velocity.y = sensor->w()[1];
  gyro.angular_velocity.z = sensor->w()[2];
  publisher.publish(gyro);
}

void BodyRosItem::updateAccelSensor(AccelerationSensor* sensor, ros::Publisher& publisher)
{
  sensor_msgs::Imu accel;
  accel.header.stamp.fromSec(controllerTarget->currentTime());
  accel.header.frame_id = sensor->name();
  accel.linear_acceleration.x = sensor->dv()[0];
  accel.linear_acceleration.y = sensor->dv()[1];
  accel.linear_acceleration.z = sensor->dv()[2];
  publisher.publish(accel);
}

void BodyRosItem::updateVisionSensor(Camera* sensor, image_transport::Publisher& publisher)
{
  sensor_msgs::Image vision;
  vision.header.stamp.fromSec(controllerTarget->currentTime());
  vision.header.frame_id = sensor->name();
  vision.height = sensor->image().height();
  vision.width = sensor->image().width();
  if (sensor->image().numComponents() == 3)
    vision.encoding = sensor_msgs::image_encodings::RGB8;
  else if (sensor->image().numComponents() == 1)
    vision.encoding = sensor_msgs::image_encodings::MONO8;
  else {
    ROS_WARN("unsupported image component number: %i", sensor->image().numComponents());
  }
  vision.is_bigendian = 0;
  vision.step = sensor->image().width() * sensor->image().numComponents();
  vision.data.resize(vision.step * vision.height);
  std::memcpy(&(vision.data[0]), &(sensor->image().pixels()[0]), vision.step * vision.height);
  publisher.publish(vision);
}

void BodyRosItem::updateRangeVisionSensor(RangeCamera* sensor, ros::Publisher& publisher)
{
  sensor_msgs::PointCloud2 range;
  range.header.stamp.fromSec(controllerTarget->currentTime());
  range.header.frame_id = sensor->name();
  range.width = sensor->resolutionX();
  range.height = sensor->resolutionY();
  range.is_bigendian = false;
  range.is_dense = true;
  range.row_step = range.point_step * range.width;
  if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
    range.fields.resize(6);
    range.fields[3].name = "rgb";
    range.fields[3].offset = 12;
    range.fields[3].count = 1;
    range.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    /*
    range.fields[3].name = "r";
    range.fields[3].offset = 12;
    range.fields[3].datatype = sensor_msgs::PointField::UINT8;
    range.fields[3].count = 1;
    range.fields[4].name = "g";
    range.fields[4].offset = 13;
    range.fields[4].datatype = sensor_msgs::PointField::UINT8;
    range.fields[4].count = 1;
    range.fields[5].name = "b";
    range.fields[5].offset = 14;
    range.fields[5].datatype = sensor_msgs::PointField::UINT8;
    range.fields[5].count = 1;
    */
    range.point_step = 16;
  } else {
    range.fields.resize(3);
    range.point_step = 12;
  }
  range.fields[0].name = "x";
  range.fields[0].offset = 0;
  range.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  range.fields[0].count = 4;
  range.fields[1].name = "y";
  range.fields[1].offset = 4;
  range.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  range.fields[1].count = 4;
  range.fields[2].name = "z";
  range.fields[2].offset = 8;
  range.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  range.fields[2].count = 4;
  const std::vector<Vector3f>& points = sensor->constPoints();
  const unsigned char* pixels = sensor->constImage().pixels();
  range.data.resize(points.size() * range.point_step);
  unsigned char* dst = (unsigned char*)&(range.data[0]);
  for (size_t j = 0; j < points.size(); ++j) {
    float x = points[j].x();
    float y = - points[j].y();
    float z = - points[j].z();
    std::memcpy(&dst[0], &x, 4);
    std::memcpy(&dst[4], &y, 4);
    std::memcpy(&dst[8], &z, 4);
    if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
      dst[14] = *pixels++;
      dst[13] = *pixels++;
      dst[12] = *pixels++;
      dst[15] = 0;
    }
    dst += range.point_step;
  }
  publisher.publish(range);
}

void BodyRosItem::updateRangeSensor(RangeSensor* sensor, ros::Publisher& publisher)
{
  sensor_msgs::LaserScan range;
  range.header.stamp.fromSec(controllerTarget->currentTime());
  range.header.frame_id = sensor->name();
  range.range_max = sensor->maxDistance();
  range.range_min = sensor->minDistance();
  if (sensor->yawRange() == 0.0) {
    range.angle_max = sensor->pitchRange()/2.0;
    range.angle_min = -sensor->pitchRange()/2.0;
    range.angle_increment = sensor->pitchRange() / ((double)sensor->pitchResolution());
  } else {
    range.angle_max = sensor->yawRange()/2.0;
    range.angle_min = -sensor->yawRange()/2.0;
    range.angle_increment = sensor->yawRange() / ((double)sensor->yawResolution());
  }
  range.ranges.resize(sensor->rangeData().size());
  //range.intensities.resize(sensor->rangeData().size());
  for (size_t j = 0; j < sensor->rangeData().size(); ++j) {
    range.ranges[j] = sensor->rangeData()[j];
    //range.intensities[j] = -900000;
  }
  publisher.publish(range);
}

void BodyRosItem::input()
{
  
}

void BodyRosItem::output()
{
  
}

void BodyRosItem::stop_publish()
{
  size_t i;

  for (i = 0; i < force_sensor_publishers_.size(); i++) {
    force_sensor_publishers_[i].shutdown();
  }

  for (i = 0; i < rate_gyro_sensor_publishers_.size(); i++) {
    rate_gyro_sensor_publishers_[i].shutdown();
  }

  for (i = 0; i < accel_sensor_publishers_.size(); i++) {
    accel_sensor_publishers_[i].shutdown();
  }

  for (i = 0; i < vision_sensor_publishers_.size(); i++) {
    vision_sensor_publishers_[i].shutdown();
  }

  for (i = 0; i < range_vision_sensor_publishers_.size(); i++) {
    range_vision_sensor_publishers_[i].shutdown();
  }

  for (i = 0; i < range_sensor_publishers_.size(); i++) {
    range_sensor_publishers_[i].shutdown();
  }

  return;
}

void BodyRosItem::stop()
{
  if (ros::ok()) {
    stop_publish();

    if (async_ros_spin_) {
      async_ros_spin_->stop();
    }

    if (rosnode_) {
      rosnode_->shutdown();
    }
  }

  return;
}

