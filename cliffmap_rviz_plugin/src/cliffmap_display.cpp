/*
 * cliffmap_display.cpp
 *
 *  Created on: Mar 1, 2019
 *      Author: ksatyaki
 */

#include "cliffmap_display.h"

namespace cliffmap_rviz_plugin {

CLiFFMapDisplay::CLiFFMapDisplay() {
  color_property_ = new rviz::ColorProperty(
      "Color", QColor(204, 51, 204), "Color to draw the acceleration arrows.",
      this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
  arrow_size_property_ =
      new rviz::FloatProperty("Size", 1.0, "Choose a value between 0.0 and 1.0",
                              this, SLOT(updateArrowSize()));
}

void CLiFFMapDisplay::onInitialize() { MFDClass::onInitialize(); }

// Clear the visuals by deleting their objects.
void CLiFFMapDisplay::reset() { MFDClass::reset(); }

void CLiFFMapDisplay::updateArrowSize() {
  visual_->setArrowSize(arrow_size_property_->getFloat());
}

// Set the current color and alpha values for each visual.
void CLiFFMapDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual_->setColor(color.r, color.g, color.b, alpha);
}

void CLiFFMapDisplay::processMessage(
    const cliffmap_ros::CLiFFMapMsg::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  boost::shared_ptr<CLiFFMapVisual> visual;
  visual.reset(new CLiFFMapVisual(context_->getSceneManager(), scene_node_));

  // Now set or update the contents of the chosen visual.
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visual_ = visual;
}

} /* namespace cliffmap_rviz_plugin */

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cliffmap_rviz_plugin::CLiFFMapDisplay, rviz::Display)
