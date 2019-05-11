//
// Created by Siukeung on 19-5-11.
//

#ifndef DISTRIBUTED_MAPPER_MAP_DRAWER_H
#define DISTRIBUTED_MAPPER_MAP_DRAWER_H
#include <pangolin/pangolin.h>
#include <mutex>
#include "distributed_mapper/distributed_mapper.h"
using namespace std;
using namespace gtsam;
namespace distributed_mapper{
    class MapDrawer {
    public:
        MapDrawer();

        void Run();

        void setDistMappers(const std::vector<boost::shared_ptr<DistributedMapper> > &dist_mappers);

    protected:
        vector<DistributedMapper> distMappers;
        vector<Point3> colorMap;

        std::mutex muDistri;
        void readColors();
        void recoverFull();

        int color_pattren[49 * 3] =
                {
                        0, 255, 255,
                        102, 205, 170,
                        180, 238, 180,
                        78, 238, 148,
                        0, 139, 69,
                        0, 255, 0,
                        255, 246, 143,
                        255, 255, 0,
                        255, 215, 0,
                        238, 180, 34,
                        238, 180, 180,
                        238, 99, 99,
                        139, 58, 58,
                        238, 121, 66,
                        205, 133, 63,
                        238, 118, 33,
                        238, 44, 44,
                        255, 64, 64,
                        238, 149, 114,
                        238, 154, 0,
                        238, 64, 0,
                        238, 0, 0,
                        238, 18, 137,
                        238, 106, 167,
                        238, 162, 173,
                        255, 52, 179,
                        238, 0, 238,
                        145, 44, 238,
                        180, 82, 205,
                        154, 192, 205,
                        164, 211, 238,
                        108, 166, 205,
                        135, 206, 255,
                        0, 154, 205,
                        92, 172, 238,
                        30, 144, 255,
                        0, 0, 255,
                        160, 32, 240,
                        218, 112, 214,
                        255, 0, 255,
                        199, 21, 133,
                        255, 20, 147,
                        255, 69, 0,
                        255, 140, 0,
                        255, 160, 122,
                        178, 34, 34,
                        244, 164, 96,
                        50, 205, 50,
                        127, 255, 0
                };

    };
}


#endif //DISTRIBUTED_MAPPER_MAP_DRAWER_H
