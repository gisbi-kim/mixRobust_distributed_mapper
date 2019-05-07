% clear all
% close all
% clc

[graph, values] = load3DAll('fullGraph_optimized.g2o');

% outlier_edge=uint64([uint64(6989586621679009793) uint64(7061644215716937731)]); %1-based
plot3DFulcon(graph, values, spoiled_edges_id);