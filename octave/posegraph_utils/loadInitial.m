function [graph,values] = loadInitial(filename1, filename2)
% load3D reads a TORO-style 3D pose graph

import gtsam.*
% loop over lines and add vertices
graph = NonlinearFactorGraph;
values = Values;
fid = fopen(filename1);
tline = fgets(fid);
model = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.3; 0.3; 0.3]);
while ischar(tline)
    foundNode = (isempty(strfind(tline, 'VERTEX_SE3:QUAT '))==0);
    if foundNode == 1
        vertex_line= textscan(tline, '%s %u64 %f %f %f %f %f %f %f');
        t = gtsam.Point3(vertex_line{3}, vertex_line{4},vertex_line{5});
        R = gtsam.Rot3.Quaternion(vertex_line{9}, vertex_line{6}, vertex_line{7}, vertex_line{8});
        values.insert(vertex_line{2}, gtsam.Pose3(R, t));
    end
    foundEdge = ( isempty(strfind(tline, 'EDGE_SE3:QUAT '))==0 );
        if foundEdge==1
            edge_line = textscan(tline, '%s %u64 %u64 %f %f %f %f %f %f %f    %f %f %f %f %f %f    %f %f %f %f %f   %f %f %f %f   %f %f %f    %f %f    %f');
            % model = gtsam.noiseModel.Diagonal.SqrtInformation([sqrt(I44); sqrt(I55); sqrt(I66); sqrt(I11); sqrt( I22); sqrt(I33)])
             t = gtsam.Point3(edge_line{4}, edge_line{5}, edge_line{6});
             R = gtsam.Rot3.Quaternion(edge_line{10}, edge_line{7}, edge_line{8}, edge_line{9});
             dpose = gtsam.Pose3(R, t);
             graph.add(BetweenFactorPose3(edge_line{2}, edge_line{3}, dpose, model));
        end
 tline = fgets(fid);
end

fid = fopen(filename2);
tline = fgets(fid);
model = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.3; 0.3; 0.3]);
while ischar(tline)
    foundNode = (isempty(strfind(tline, 'VERTEX_SE3:QUAT '))==0);
    if foundNode == 1
        vertex_line= textscan(tline, '%s %u64 %f %f %f %f %f %f %f');
        t = gtsam.Point3(vertex_line{3}, vertex_line{4},vertex_line{5});
        R = gtsam.Rot3.Quaternion(vertex_line{9}, vertex_line{6}, vertex_line{7}, vertex_line{8});
        values.insert(vertex_line{2}, gtsam.Pose3(R, t));
    end
    foundEdge = ( isempty(strfind(tline, 'EDGE_SE3:QUAT '))==0 );
        if foundEdge==1
            edge_line = textscan(tline, '%s %u64 %u64 %f %f %f %f %f %f %f    %f %f %f %f %f %f    %f %f %f %f %f   %f %f %f %f   %f %f %f    %f %f    %f');
            % model = gtsam.noiseModel.Diagonal.SqrtInformation([sqrt(I44); sqrt(I55); sqrt(I66); sqrt(I11); sqrt( I22); sqrt(I33)])
             if symbolChr(edge_line{2}) == symbolChr(edge_line{3})
            t = gtsam.Point3(edge_line{4}, edge_line{5}, edge_line{6});
             R = gtsam.Rot3.Quaternion(edge_line{10}, edge_line{7}, edge_line{8}, edge_line{9});
             dpose = gtsam.Pose3(R, t);
             graph.add(BetweenFactorPose3(edge_line{2}, edge_line{3}, dpose, model));
             end
        end
 tline = fgets(fid);
end

end

