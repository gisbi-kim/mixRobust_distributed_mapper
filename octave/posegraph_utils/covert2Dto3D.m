function [ robot_poses, robot_measurements, robot_edges_id, poses, measurements, edges_id, trajectory_size] = covert2Dto3D( input_file, robots_offsets )
 nodeId = 0;
 edgeId = 0;
%% 读取2维SE2数据
fid_g2o = fopen(input_file, 'r');
tline = fgets(fid_g2o);
  while ischar(tline)
     foundNode = ( isempty(strfind(tline, 'VERTEX_SE2'))==0 );
    %% 读取顶点
     if foundNode==1
         nodeId = nodeId+1;
         [name, id, x, y, th] = strread(tline, '%s %d %f %f %f');
         poses(nodeId).t = [x, y ,0]';
         matrix4 = createRotationOz(th);
         poses(nodeId).R = matrix4(1:3,1:3);
     end
       foundEdge = ( isempty(strfind(tline, 'EDGE_SE2'))==0 );
     %% EDGE line
     if foundEdge ==1
         edgeId = edgeId + 1;
         [name, id1, id2, dx, dy, dth, ...
             I11, I12, I13,...
                     I22, I23,...
                             I33]=...
           strread(tline, '%s %d %d %f %f %f %f %f %f %f %f %f ');
        edges_id(edgeId,:) = [id1+1, id2+1];
        measurements.between(edgeId).t = [dx dy 0]';
        matrix4 = createRotationOz(dth);
        measurements.between(edgeId).R =  matrix4(1:3,1:3);
        measurements.between(edgeId).Info = ...
            [I11 I12  0   0     0     0  ;
             I12 I22   0     0     0     0  ;
               0     0   100000000   0     0     0  ;
               0     0     0   100000000   0     0  ;
               0     0     0     0   100000000   0  ;
               0     0     0     0     0   I33];
             if norm(measurements.between(edgeId).Info - measurements.between(edgeId).Info',inf)>1e-5
                error('Nonsymmetric information matrix');
             end
     end
     tline = fgets(fid_g2o);
  end
  fclose(fid_g2o);
  
  %% spilt the data
  trajectory_size = floor(nodeId/2);
  edges_id1 = uint64([]);
  edges_id2 = uint64([]);
  %% spilt measurements and edges_id
  meaId1 = 0;
  meaId2 = 0;
  for along_all = 1 : size(edges_id, 1)
      edgeID = edges_id(along_all, :);
      if edgeID(1) <= trajectory_size
          meaId1 = meaId1+1;
          measurements1.between(meaId1).R = measurements.between(along_all).R;
          measurements1.between(meaId1).t = measurements.between(along_all).t;
          measurements1.between(meaId1).Info = measurements.between(along_all).Info;
          if edgeID(2) <= trajectory_size
              edges_id1(end+1, :) = [uint64(robots_offsets{1}+edgeID(1)), uint64(robots_offsets{1}+edgeID(2))];
          else
              edges_id1(end+1, :) = [uint64(robots_offsets{1}+edgeID(1)), uint64(robots_offsets{2}+edgeID(2)-trajectory_size)];
              meaId2 = meaId2+1;
              measurements2.between(meaId2).R = measurements.between(along_all).R;
              measurements2.between(meaId2).t = measurements.between(along_all).t;
              measurements2.between(meaId2).Info = measurements.between(along_all).Info;
              edges_id2(end+1, :) = [uint64(robots_offsets{1}+edgeID(1)), uint64(robots_offsets{2}+edgeID(2)-trajectory_size)];
          end
      else
          meaId2 = meaId2+1;
          measurements2.between(meaId2).R = measurements.between(along_all).R;
          measurements2.between(meaId2).t = measurements.between(along_all).t;
          measurements2.between(meaId2).Info = measurements.between(along_all).Info;
          if edgeID(2) <= trajectory_size
               edges_id2(end+1, :) = [uint64(robots_offsets{1}+edgeID(2)), uint64(robots_offsets{2}+edgeID(1)-trajectory_size)];
               meaId1 = meaId1+1;
               measurements1.between(meaId1).R = measurements.between(along_all).R;
               measurements1.between(meaId1).t = measurements.between(along_all).t;
               measurements1.between(meaId1).Info = measurements.between(along_all).Info;
               edges_id1(end+1, :) = [uint64(robots_offsets{1}+edgeID(2)), uint64(robots_offsets{2}+edgeID(1)-trajectory_size)];
          else
               edges_id2(end+1, :) = [uint64(robots_offsets{2}+edgeID(1)-trajectory_size), uint64(robots_offsets{2}+edgeID(2)-trajectory_size)];
          end
      end
end
  %%    poses spilt
  posId1 = 0;
  posId2 = 0;
      for index = 1:size(poses,2)
          if index <= trajectory_size
              posId1 = posId1+1;
              poses1(posId1).t = poses(index).t;
              poses1(posId1).R = poses(index).R;
          else
              posId2 = posId2+1;
              poses2(posId2).t = poses(index).t;
              poses2(posId2).R = poses(index).R;
          end
      end
%% aggregate
robot_poses{1} = poses1;
robot_poses{2} = poses2;
robot_measurements{1} = measurements1;
robot_measurements{2} = measurements2;
robot_edges_id{1} = edges_id1;
robot_edges_id{2} = edges_id2;

end

