// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H
#define DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H

#include "distributed_mapper/distributed_mapper.h"
#include "graph_utils/graph_types.h"
#include "robot_measurements/robot_local_map.h"
#include "global_map/global_map.h"

/** \namespace distributed_pcm
 *  \brief This namespace encapsulates the pairwise consistency maximization in a distributed setup.
 */
namespace distributed_pcm {
    /** \class DistributedPCM
     * \brief Class computing the pairwise consistency maximization with partial information.
     */
    class DistributedPCM {
        public:
        /** \brief Constructor
         */
        DistributedPCM(){};

        /**
         * \brief Function that solves the global maps according to the current constraints with perfect information
         *
         * @param dist_mappers is the different distributed mappers in the system (one by robot)
         * @param graph_and_values is the collection of factors of all graph used for evaluation
         * @returns size of the maximum clique of pairwise consistent measurements
         */
        static int solveCentralized(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                const double& confidence_probability, const bool& use_covariance, const bool& use_pcmHeu);

        private:

        static void fillInRequiredInformation(std::vector<graph_utils::LoopClosures>& separators_by_robot,
                                       std::vector<graph_utils::Transforms>& transforms_by_robot,
                                       std::map<std::pair<char, char>,graph_utils::Transforms>& separators_transforms_by_pair,
                                       const std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                                       const bool& use_covariance);

        static int executePCM(const int& roboti, const int& robotj, const std::vector<graph_utils::Transforms>& transforms_by_robot,
                                const std::vector<graph_utils::LoopClosures>& separators_by_robot,
                                const std::map<std::pair<char, char>,graph_utils::Transforms>& separators_transforms_by_pair,
                                std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                                const double& confidence_probability, const bool& use_pcmHeu);

        static bool isSeparatorToBeRejected(const std::vector<int>& max_clique, const int& separtor_id, const graph_utils::Transforms& separators_transforms,
                                                     const graph_utils::LoopClosures& loop_closures, boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper);

    };
}

#endif //DISTRIBUTED_MAPPER_DISTRIBUTED_PCM_H
