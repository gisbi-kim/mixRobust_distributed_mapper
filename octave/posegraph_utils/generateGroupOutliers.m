function [ spoiled_measurements, spoiled_edges_id ,loopclosures] = generateGroupOutliers( robot_edges_id, robots_offsets,  trajectory_size, outliers,  groupSize)

edges_id1 = robot_edges_id{1};
edges_id2 = robot_edges_id{2};
%% select loopclosures
loopclosure_indexs = ismember(edges_id1, edges_id2, 'rows');
loopclosures = uint64([ ]);
spoiled_edges_id = uint64([ ]);
for loop_index = 1:size(loopclosure_indexs,1)
    if loopclosure_indexs(loop_index) ~= 0 
        loopclosures(end+1,:) = edges_id1(loop_index, :);
    end
end
%% create additional outlier loopclosures
information_matrix =[42 0 0 0 0 0 ;
                                       0 42 0 0 0 0;
                                       0 0 100000 0 0 0;
                                       0 0 0 100000 0 0;
                                       0 0 0 0 100000 0 ;
                                       0 0 0 0 0 84];
measure_id = 0;
%% create group outliers
for i = 1 : outliers
    v1 = randi(trajectory_size-groupSize);
    v2 = randi(trajectory_size-groupSize);
    %% create gaussian loop closure constraint
    dx = normrnd(0, 0.3);
    dy = normrnd(0, 0.3);
    sigma = 10*pi/180;
    dth = normrnd(0, sigma);
    matrix4 = createRotationOz(dth);
    dR = matrix4(1:3,1:3);
    for j = 1:groupSize
        if ismember([uint64(robots_offsets{1}+v1), uint64(robots_offsets{2}+v2)], loopclosures, 'rows')~=1
        measure_id =  measure_id+1;
        spoiled_edges_id(end+1,:) = [uint64(robots_offsets{1}+v1), uint64(robots_offsets{2}+v2)];
        spoiled_measurements.between(measure_id).t = [dx, dy 0]';
        spoiled_measurements.between(measure_id).R = dR;
        spoiled_measurements.between(measure_id).Info = information_matrix;
        end
        v1 = v1+1;
        v2 = v2+1;
    end
end

%% create random outliers
for i=1:20
    v1 = randi(trajectory_size-groupSize);
    v2 = randi(trajectory_size-groupSize);
    %% create gaussian loop closure constraint
    dx = normrnd(0, 0.3);
    dy = normrnd(0, 0.3);
    sigma = 10*pi/180;
    dth = normrnd(0, sigma);
    matrix4 = createRotationOz(dth);
    dR = matrix4(1:3,1:3);
    if ismember([uint64(robots_offsets{1}+v1), uint64(robots_offsets{2}+v2)], loopclosures, 'rows')~=1
        measure_id =  measure_id+1;
        spoiled_edges_id(end+1,:) = [uint64(robots_offsets{1}+v1), uint64(robots_offsets{2}+v2)];
        spoiled_measurements.between(measure_id).t = [dx, dy 0]';
        spoiled_measurements.between(measure_id).R = dR;
        spoiled_measurements.between(measure_id).Info = information_matrix;
   end
end
end

