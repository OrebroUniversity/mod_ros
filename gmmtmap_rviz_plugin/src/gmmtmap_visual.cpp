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

#include "gmmtmap_visual.h"
#include <ros/console.h>

namespace gmmtmap_rviz_plugin {

const std::array<float, 3> arrow_props = {0.1f, 0.25f, 0.25f};

GMMTMapVisual::GMMTMapVisual(Ogre::SceneManager *scene_manager,
                             Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the GMMTmap's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  gmmtmap_arrows_.clear();
}

GMMTMapVisual::~GMMTMapVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void GMMTMapVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void GMMTMapVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void GMMTMapVisual::setShowVariance(bool value) {
  this->show_variance_ = value;
}

// Color is passed through to the Arrow object.
void GMMTMapVisual::setColor(float r, float g, float b, float a) {
  color_[0] = r;
  color_[1] = g;
  color_[2] = b;
  color_[3] = a;

  for (auto &arrow : gmmtmap_arrows_) {
    arrow->setColor(color_[0], color_[1], color_[2], color_[3]);
  }
}

void GMMTMapVisual::setArrowSize(float size_multiplier) {
  size_multiplier_ = size_multiplier;
  if (size_multiplier < 0.0 || size_multiplier > 10.0)
    return;

  for (size_t i = 0; i < gmmtmap_arrows_.size(); i++) {
    auto &arrow = *gmmtmap_arrows_[i];
    auto &size = arrow_lengths_[i];
    arrow.set(arrow_lengths_[i], arrow_props[0] * size_multiplier_,
              arrow_props[1] * size_multiplier_,
              arrow_props[2] * size_multiplier_);
  }
}

void GMMTMapVisual::setMessage(const gmmtmap_ros::GMMTMapMsg::ConstPtr &msg) {
  arrow_lengths_.clear();
  gmmtmap_arrows_.clear();

  for (const auto &pattern : msg->clusters) {

    for (size_t i = 0; i < pattern.mean.size() - 1; i++) {

      auto &this_dist = pattern.mean[i];
      auto &next_dist = pattern.mean[i + 1];

      const auto &x = this_dist.x;
      const auto &y = this_dist.y;

      auto distance = [](const geometry_msgs::Point &a,
                         const geometry_msgs::Point &b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
      };

      auto getangle = [](const geometry_msgs::Point &a,
                         const geometry_msgs::Point &b) {
        return std::atan2(a.y - b.y, a.x - b.x);
      };

      auto shaft_length = distance(this_dist, next_dist);
      auto theta = getangle(next_dist, this_dist);

      // These are the default parameters to rviz::Arrow()
      // float shaft_length=1.0f, float shaft_diameter=0.1f,
      // float head_length=0.3f, float head_diameter=0.2f
      ArrowPtr this_arrow = boost::make_shared<rviz::Arrow>(
          scene_manager_, frame_node_, shaft_length, arrow_props[0],
          arrow_props[1], arrow_props[2]);
      this_arrow->setPosition(Ogre::Vector3(x, y, 0.1));
      Ogre::Quaternion q;
      q.FromAngleAxis(Ogre::Radian(theta), Ogre::Vector3::UNIT_Z);
      this_arrow->setOrientation(
          q * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
      this_arrow->setColor(color_[0], color_[1], color_[2], color_[3]);

      gmmtmap_arrows_.push_back(this_arrow);
      arrow_lengths_.push_back(shaft_length);
    }
  }

  ROS_INFO_STREAM("[GMMTMapVisual]: Received a new GMMT-map containing "
                  << gmmtmap_arrows_.size() << " distributions in total.");
}

} /* namespace gmmtmap_rviz_plugin */
