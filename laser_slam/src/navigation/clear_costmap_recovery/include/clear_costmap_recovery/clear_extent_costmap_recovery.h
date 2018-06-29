/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef CLEAR_EXTENT_COSTMAP_RECOVERY_H_
#define CLEAR_EXTENT_COSTMAP_RECOVERY_H_
#include <clear_costmap_recovery/clear_costmap_recovery.h>

namespace clear_costmap_recovery{
  /**
   * @class ClearExtentCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class ClearExtentCostmapRecovery : public ClearCostmapRecovery
  {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param  
       * @return 
       */
      ClearExtentCostmapRecovery();
      /**
       * @brief  Initialization function for the ClearCostmapRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack
       * @param local_costmap A pointer to the local_costmap used by the navigation stack
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);
      /**
       * @brief  Run the ClearCostmapRecovery recovery behavior. Reverts the
       * costmap to the static map outside of a user-specified window and
       * clears unknown space around the robot.
       */
      void runBehavior();
    private:
      void clearMapWithLocalArea(boost::shared_ptr<costmap_2d::CostmapLayer> costmap,double pose_x, double pose_y,double clear_width,double clear_height);
	  void clearglobal(costmap_2d::Costmap2DROS* costmap,double clear_width,double clear_height);
      double clear_global_width_;
      double clear_global_height_;
  };
};
#endif  
