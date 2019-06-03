clear all
close all
clc

%% settings
ori_folder = horzcat(pwd, '/../test_data/datasets/city10000/originalDataset/part_city10000.g2o');
spilt_folder = horzcat(pwd, '/../test_data/datasets/city10000/PCMdata/');
mkdir(spilt_folder);
number_of_robots = 2;
id_offset = 96;
addpath(genpath('./posegraph_utils'));

for robot=1:number_of_robots
    robots_offsets{robot} = bitshift(uint64(robot+id_offset), 56); % GTSAM format
end
%% Generate file names
file_names = {};  
for robot=1:number_of_robots
    file_names{end+1} = horzcat(spilt_folder,num2str(robot-1),'.g2o');
end

%% spilt city10000
 [ robot_poses, robot_measurements, robot_edges_id, poses, measurements, edges_id, trajectory_size]...
     = covert2Dto3D( ori_folder, robots_offsets );
 
 %% write full graph
 fullfile_name = horzcat(spilt_folder,'fullgraph_without_outliers.g2o');
 writeG2oDataset3D(fullfile_name, measurements, edges_id, poses, 0);
 
 %% write spilt graph
 writeG2oDataset3D(file_names{1}, robot_measurements{1}, robot_edges_id{1}, robot_poses{1}, robots_offsets{1});
 writeG2oDataset3D(file_names{2}, robot_measurements{2}, robot_edges_id{2}, robot_poses{2}, robots_offsets{2});
 
 %% generate inter loopclosure outliers
 [ spoiled_measurements, spoiled_edges_id ,loopclosures] = generateGroupOutliers( robot_edges_id, robots_offsets, ...
    trajectory_size, 10,  10, 80);
 writeG2oDataset3D(file_names{1}, spoiled_measurements, spoiled_edges_id, [], 0, 1 );
 writeG2oDataset3D(file_names{2}, spoiled_measurements, spoiled_edges_id, [], 0, 1 );