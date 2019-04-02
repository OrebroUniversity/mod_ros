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

#include <array>
#include <boost/make_shared.hpp>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <rviz/ogre_helpers/arrow.h>

#include <stefmap_ros/STeFMapMsg.h>

namespace Ogre {
class Vector3;
class Quaternion;
}

namespace rviz {
class Arrow;
}

namespace stefmap_rviz_plugin {

class STeFMapVisual {
 public:
  STeFMapVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* parent_node);
  virtual ~STeFMapVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const stefmap_ros::STeFMapMsg::ConstPtr& msg);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way STeFMapVisual is only
  // responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the cliffmap message.
  void setColor(float r, float g, float b, float a);

  void setArrowSize(float size_multiplier);

 private:
  float size_multiplier_{1.0};

  float color_[4] {0.0, 0.35, 0.35, 1.0};

  std::vector<float> arrow_speeds_;

  // The object implementing the actual arrow shape
   std::vector<boost::shared_ptr<rviz::Arrow>> stefmap_arrows_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Imu message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};

} /* namespace stefmap_rviz_plugin */
