/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of cliffmap_rviz_plugin.
 *
 *   cliffmap_rviz_plugin is free software: you can redistribute it and/or
 * modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   cliffmap_rviz_plugin is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with cliffmap_rviz_plugin.  If not, see
 * <https://www.gnu.org/licenses/>.
 */

#include "cliffmap_visual.h"

namespace cliffmap_rviz_plugin {

CLiFFMapVisual::CLiFFMapVisual(Ogre::SceneManager* scene_manager,
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

  cliffmap_arrows_.reset(new std::vector<rviz::Arrow>());
}

CLiFFMapVisual::~CLiFFMapVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void CLiFFMapVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void CLiFFMapVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the Arrow object.
void CLiFFMapVisual::setColor(float r, float g, float b, float a) {
  color_[0] = r;
  color_[1] = g;
  color_[2] = b;

  for (auto& arrow : *cliffmap_arrows_) {
    arrow.setColor(color_[0], color_[1], color_[2], color_[3]);
  }
}

void CLiFFMapVisual::setArrowSize(float size_multiplier) {
  size_multiplier_ = size_multiplier;
  if (size_multiplier < 0.0 || size_multiplier > 1.0) return;

  for (size_t i = 0; i < cliffmap_arrows_->size(); i++) {
    auto& arrow = (*cliffmap_arrows_)[i];
    auto& speed = arrow_speeds_[i];

    Ogre::Vector3 scale(size_multiplier * speed / 5.0, size_multiplier * 0.035,
                        size_multiplier * 0.015);
    arrow.setScale(scale);
  }
}

void CLiFFMapVisual::setMessage(
    const cliffmap_ros::CLiFFMapMsg::ConstPtr& msg) {
  arrow_speeds_.clear();
  cliffmap_arrows_->clear();

  for (const auto& location : msg->locations) {
    const auto& x = location.position[0];
    const auto& y = location.position[1];

    // Alpha depends on trust factor of cliffmap location
    color_[3] = 0.5 + (location.q * 0.25) + (location.p * 0.25);

    for (const auto& distribution : location.distributions) {
      const auto& speed = distribution.mean[1];
      const auto& theta = distribution.mean[0];

      cliffmap_arrows_->push_back(rviz::Arrow(scene_manager_, frame_node_));
      arrow_speeds_.push_back(speed);

      auto& this_arrow = cliffmap_arrows_->back();

      this_arrow.setPosition(Ogre::Vector3(x, y, 0.0));
      this_arrow.setOrientation(
          Ogre::Quaternion(cos(theta / 2.0), 0.0, 0.0, sin(theta / 2.0)));
      this_arrow.setColor(color_[0], color_[1], color_[2], color_[3]);
      Ogre::Vector3 scale(size_multiplier_ * speed / 5.0,
                          size_multiplier_ * 0.035, size_multiplier_ * 0.015);
      this_arrow.setScale(scale);
    }
  }
}

} /* namespace cliffmap_rviz_plugin */
