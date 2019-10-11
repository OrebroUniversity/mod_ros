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

#include "stefmap_visual.h"
#include <ros/console.h>

namespace stefmap_rviz_plugin {

STeFMapVisual::STeFMapVisual(Ogre::SceneManager* scene_manager,
                             Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the cliffmap's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  stefmap_arrows_.clear();
}

STeFMapVisual::~STeFMapVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void STeFMapVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void STeFMapVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the Arrow object.
void STeFMapVisual::setColor(float r, float g, float b, float a) {
  color_[0] = r;
  color_[1] = g;
  color_[2] = b;

  for (auto& arrow : stefmap_arrows_) {
    arrow->setColor(color_[0], color_[1], color_[2], color_[3]);
  }
}

void STeFMapVisual::setArrowSize(float size_multiplier) {
  size_multiplier_ = size_multiplier;
  if (size_multiplier < 0.0 || size_multiplier > 10.0) return;

  for (size_t i = 0; i < stefmap_arrows_.size(); i++) {
    auto& arrow = *stefmap_arrows_[i];
    arrow.setScale(
        Ogre::Vector3(size_multiplier, size_multiplier, size_multiplier));
  }
}

void STeFMapVisual::setMessage(const stefmap_ros::STeFMapMsg::ConstPtr& msg) {
  stefmap_arrows_.clear();

  for (const auto& location : msg->cells) {
    const auto& x = location.x;
    const auto& y = location.y;

    double norm = location.probabilities[0] * location.probabilities[0] +
                  location.probabilities[1] * location.probabilities[1] +
                  location.probabilities[2] * location.probabilities[2] +
                  location.probabilities[3] * location.probabilities[3] +
                  location.probabilities[4] * location.probabilities[4] +
                  location.probabilities[5] * location.probabilities[5] +
                  location.probabilities[6] * location.probabilities[6] +
                  location.probabilities[7] * location.probabilities[7];
    if(norm == 0.0)
      continue;
//    for (int i = 0; i < 8; i++) {
//      if(location.probabilities[i] < 1e-1)
//        continue;
      const auto& theta = location.best_angle;

      color_[3] = 1.0;
      // These are the default parameters to rviz::Arrow()
      // float shaft_length=1.0f, float shaft_diameter=0.1f,
      // float head_length=0.3f, float head_diameter=0.2f
      boost::shared_ptr<rviz::Arrow> this_arrow =
          boost::make_shared<rviz::Arrow>(scene_manager_, frame_node_, 0.001f,
                                          0.005f, 0.5f, 0.25f);
      this_arrow->setPosition(Ogre::Vector3(x, y, 0.1));
      Ogre::Quaternion q;
      q.FromAngleAxis(Ogre::Degree(theta), Ogre::Vector3::UNIT_Z);
      this_arrow->setOrientation(
          q * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
      this_arrow->setColor(color_[0], color_[1], color_[2], color_[3]);
      this_arrow->setScale(
          Ogre::Vector3(size_multiplier_, size_multiplier_, size_multiplier_));
      stefmap_arrows_.push_back(this_arrow);
    }
//  }

  ROS_INFO_STREAM("[STeFMapVisual]: Received a new STeF containing "
                  << stefmap_arrows_.size() << " arrows in total.");
}

} /* namespace stefmap_rviz_plugin */
