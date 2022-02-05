/*
 * Planner.cpp
 *
 *  Created on: 05 Feb 2022
 *      Author: moritzschueler
 */

#include <arp/Planner.hpp>

namespace arp {

Planner::Planner(ros::NodeHandle& nh)
    : nh_(&nh)
{
  found_ = false; // always assume no found path  

  
}

bool Planner::plan(){
    // do shit
    return false;
}

} // namespace arp



