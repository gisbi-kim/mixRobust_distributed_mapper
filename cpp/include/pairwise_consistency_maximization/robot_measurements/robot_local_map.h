// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef ROBOT_LOCAL_MAP_H
#define ROBOT_LOCAL_MAP_H

#include "robot_measurements.h"

namespace robot_measurements {
    /** \class RobotLocalMap
     * Class that contains the local map of a single robot.
     */ 
    class RobotLocalMap : public RobotMeasurements {
      public:
        /**
         * \brief Constructor
         * @param file_name Name of the file containing the robot measurements.
         */
        RobotLocalMap(const graph_utils::Transforms& transforms,
                      const graph_utils::LoopClosures& loop_closures);

        /*
         * Accessors
         */

        /** \brief Accessor
         * @return the robot trajectory
         */
        const graph_utils::Trajectory& getTrajectory() const;

      private:
        graph_utils::Trajectory trajectory_; ///< Local trajectory of the robot.
    };          
}

#endif