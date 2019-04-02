/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of stefmap_rviz_plugin.
 *
 *   stefmap_rviz_plugin is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   stefmap_rviz_plugin is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with stefmap_rviz_plugin.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef Q_MOC_RUN

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

#include <stefmap_ros/STeFMapMsg.h>
#include "stefmap_visual.h"

#endif

namespace stefmap_rviz_plugin {

class STeFMapDisplay
    : public rviz::MessageFilterDisplay<stefmap_ros::STeFMapMsg> {
  Q_OBJECT

 protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties.
 private Q_SLOTS:
  void updateColorAndAlpha();
  void updateArrowSize();

  // Function to handle an incoming ROS message.
 private:
  void processMessage(const stefmap_ros::STeFMapMsg::ConstPtr& msg);

  boost::shared_ptr<STeFMapVisual> visual_;

  // User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* arrow_size_property_;

 public:
  STeFMapDisplay();
  virtual ~STeFMapDisplay() {}
};

} /* namespace stefmap_rviz_plugin */
