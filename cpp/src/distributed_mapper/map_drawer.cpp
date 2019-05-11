//
// Created by Siukeung on 19-5-11.
//

#include "distributed_mapper/map_drawer.h"

namespace distributed_mapper{
    void MapDrawer::readColors() {
        for(size_t i = 0; i < 49; i++){
            Point3 singleColor(color_pattren[3*i]/(float)255, color_pattren[3*i+1]/(float)255, color_pattren[3*i+2]/(float)255);
            colorMap.push_back(singleColor);
        }
    }
    MapDrawer::MapDrawer() {
      readColors();
    }

    void MapDrawer::setDistMappers(const std::vector<boost::shared_ptr<DistributedMapper> > &dist_mappers,bool isfirst) {
        unique_lock<mutex> lock(muDistri);
        distMappers.clear();
        for(const boost::shared_ptr<DistributedMapper>& dist_mapper : dist_mappers){
            DistributedMapper distmap = *dist_mapper;
            distmap.retractPose3Global4plot(isfirst);
            distMappers.emplace_back(distmap);
        }
    }

    void MapDrawer::recoverFull() {
        unique_lock<mutex> lock(muDistri);
        int color = 0;
        size_t Max = distMappers.size();
        size_t colorStep = 49/Max;
        for(DistributedMapper& distMapper: distMappers){
            color+=colorStep;
            NonlinearFactorGraph graph = distMapper.currentGraph();
            //distMapper.retractPose3Global4plot();
            Values estimate = distMapper.currentEstimate();
            Values neighbors = distMapper.neighbors();
            char robotName = distMapper.robotName();
            for(size_t k = 0; k < graph.size(); k++){
                boost::shared_ptr<BetweenFactor<Pose3>> factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(graph[k]);
                if(factor){
                    Symbol key1 = factor->keys().at(0);
                    Symbol key2 = factor->keys().at(1);
                    char robot0 = symbolChr(key1);
                    char robot1 = symbolChr(key2);
                    Pose3 measured = factor->measured();
                    if(robot0 == robot1){
                        Pose3 formerPose = estimate.at<Pose3>(key1);
                        Pose3 latterPose = estimate.at<Pose3>(key2);
                        Point3 t1 = formerPose.translation();
                        Rot3 rot1 = formerPose.rotation();
                        Point3 t2 = latterPose.translation();
                        Rot3 rot2 = latterPose.rotation();
                        glLineWidth(3.0);
                        Point3 colorPoint = colorMap[color];
                        glColor3f(colorPoint.x(), colorPoint.y(), colorPoint.z());
                        //glColor3f(1-(float)color/Max,1-(float)color/(2*Max) , (float)color/Max);
                        glBegin(GL_LINES);
                        glVertex3f(t1.x(), t1.y(), t1.z() );
                        glVertex3f(t2.x(), t2.y(), t2.z());
                        glEnd();
                    }
                    else if(robot0 == robotName){
                        Pose3 innerPose = estimate.at<Pose3>(key1);
                        Pose3 interPose = neighbors.at<Pose3>(key2);
                        Point3 t1 = innerPose.translation();
                        Rot3 rot1 = innerPose.rotation();
                        Point3 t2 = interPose.translation();
                        Rot3 rot2 = interPose.rotation();
                        glLineWidth(1.0);
                        glColor3f(0.6f, 0.6f, 0.6f);
                        glBegin(GL_LINES);
                        glVertex3f(t1.x(), t1.y(), t1.z() );
                        glVertex3f(t2.x(), t2.y(), t2.z());
                        glEnd();
                    }else if(robot1 == robotName){
                        Pose3 innerPose = estimate.at<Pose3>(key2);
                        Pose3 interPose = neighbors.at<Pose3>(key1);
                        Point3 t1 = innerPose.translation();
                        Rot3 rot1 = innerPose.rotation();
                        Point3 t2 = interPose.translation();
                        Rot3 rot2 = interPose.rotation();
                        glLineWidth(1.0);
                        glColor3f(0.6f, 0.6f, 0.6f);
                        glBegin(GL_LINES);
                        glVertex3f(t1.x(), t1.y(), t1.z() );
                        glVertex3f(t2.x(), t2.y(), t2.z());
                        glEnd();
                    }
                }
            }
        }
    }


    void MapDrawer::Run() {

        pangolin::CreateWindowAndBind("Robots",1024,768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        // Define Camera Render Object (for view / scene browsing)
        // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
        // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
        //                观测目标位置：(0, 0, 0)
        //                观测的方位向量：(0.0,-1.0, 0.0)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,2000,2000,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,-10,100, 0,0,0,0.0,-1.0, 0.0)
        );
        // Add named OpenGL viewport to window and provide 3D Handler
        // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
        // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
        // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
        // 最后一个参数（-1024.0f/768.0f）为显示长宽比
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));
        while(!pangolin::ShouldQuit()){
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            //DrawCoordinateSystem();
            recoverFull();
            pangolin::FinishFrame();
        }
    }
}
