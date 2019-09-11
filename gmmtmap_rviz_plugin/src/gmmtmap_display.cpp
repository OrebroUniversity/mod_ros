/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of gmmtmap_rviz_plugin.
 *
 *   gmmtmap_rviz_plugin is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   gmmtmap_rviz_plugin is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with gmmtmap_rviz_plugin.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#include "gmmtmap_display.h"

namespace gmmtmap_rviz_plugin {

GMMTMapDisplay::GMMTMapDisplay() {
  color_property_ = new rviz::ColorProperty(
      "Color", QColor(204, 51, 204), "Color to draw the acceleration arrows.",
      this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
  arrow_size_property_ =
      new rviz::FloatProperty("Size", 1.0, "Choose a value between 0.0 and 1.0",
                              this, SLOT(updateArrowSize()));
  show_variance_property_ = new rviz::BoolProperty(
      "Show variance", false, "Show variance as part of GMMT viz?", this,
      SLOT(updateShowVariance()));
}

void GMMTMapDisplay::onInitialize() { MFDClass::onInitialize(); }

// Clear the visuals by deleting their objects.
void GMMTMapDisplay::reset() { MFDClass::reset(); }

void GMMTMapDisplay::updateArrowSize() {
  visual_->setArrowSize(arrow_size_property_->getFloat());
}

// Set the current color and alpha values for each visual.
void GMMTMapDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual_->setColor(color.r, color.g, color.b, alpha);
}

void GMMTMapDisplay::updateShowVariance() {
  visual_->setShowVariance(show_variance_property_->getBool());
}

void GMMTMapDisplay::processMessage(
    const gmmtmap_ros::GMMTMapMsg::ConstPtr &msg) {
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

  boost::shared_ptr<GMMTMapVisual> visual;
  visual.reset(new GMMTMapVisual(context_->getSceneManager(), scene_node_));

  Ogre::ColourValue color = color_property_->getOgreColor();
  // Now set or update the contents of the chosen visual.
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);
  visual->setColor(color.r, color.g, color.b, alpha_property_->getFloat());
  visual->setShowVariance(show_variance_property_->getBool());
  visual_ = visual;
}

} /* namespace gmmtmap_rviz_plugin */

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gmmtmap_rviz_plugin::GMMTMapDisplay, rviz::Display)
