// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>
//modified by Siukeung <siukeung9527@gmail.com>
#include "robot_measurements/robot_measurements.h"
#include "graph_utils/graph_utils_functions.h"

namespace robot_measurements {

RobotMeasurements::RobotMeasurements(const graph_utils::Transforms& transforms,
                                     const graph_utils::LoopClosures& loop_closures){
    num_poses_ = transforms.transforms.size() + 1;
    loop_closures_ = loop_closures;
    transforms_ = transforms;
    nb_degree_freedom_ = 6; // TODO: support 2D case
}

const graph_utils::Transforms& RobotMeasurements::getTransforms() const {
    return transforms_;
}

const size_t& RobotMeasurements::getNumPoses() const {
    return num_poses_;
}

const graph_utils::LoopClosures& RobotMeasurements::getLoopClosures() const {
    return loop_closures_;
}

const uint8_t& RobotMeasurements::getNbDegreeFreedom() const {
    return nb_degree_freedom_;
}

}