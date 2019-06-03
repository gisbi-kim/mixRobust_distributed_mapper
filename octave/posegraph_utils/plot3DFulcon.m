function plot3DFulcon( graph, values, outlier_loopclosures )
%plot3DFulcon 画出优化后的GRAPH
%   注意outlier_loopclosures里的id是1-based，而graph和values里的key是0-based

import gtsam.*

% plot resulted graph
for i = 0 : graph.size - 1
    factor = graph.at(i);
    if size(factor.keys) == 2
        key1 = factor.keys.at(0);
        key2 = factor.keys.at(1);
     
        try 
            pose1 = values.atPose3(key1);
            pose2 = values.atPose3(key2);
            % plot edge line
            key1_sym = symbolChr(key1);
            key2_sym = symbolChr(key2);
            key1_index = symbolIndex(key1);
            key2_index = symbolIndex(key2);
            
            if key1_sym == key2_sym && key1_sym == 'a'
                %robot a inner edges
                if key1_index == key2_index - 1 || key1_index - 1 == key2_index
                    plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'linewidth', 0.5, 'Color' ,[0.12 0.56 1])
                else
                    plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'linewidth', 1.8, 'Color' ,[0.12 0.56 1])
                end
                hold on
            elseif key1_sym == key2_sym && key1_sym == 'b'
                %robot b inner edges
                if key1_index == key2_index - 1 || key1_index - 1 == key2_index
                    plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'linewidth', 0.5, 'Color' ,[0 0.8 0])
                else
                    plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'linewidth', 1.8, 'Color' ,[0 0.8 0])
                end 
            elseif key1_sym ~= key2_sym
               %plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'linewidth', 1.8, 'Color' ,[0 0 0.55])
                % inter loopclosures
                % 判断该inter回环是否为添加进的outlier
                error_index = ismember(outlier_loopclosures, [key1+1 key2+1],  'rows');
                if isempty(find(error_index,1)) == 0
                    plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'linewidth',  1, 'Color' ,[0.8 0.78 0.78],'LineStyle', ':')
                    % 筛选出被拒绝的edge
                    for k = 1: size(error_index, 1)
                        if error_index(k) ==1
                            outlier_loopclosures(k, :)=[];
                        end
                    end
                else
                    plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'linewidth', 1.8, 'Color' ,[0 0 0.55])
                end
            end
        catch
        end
    end
    hold on
    xlim([-60 60])
    ylim([-60 60])

end
% plot refused outlier inter edge
for m = 1: size(outlier_loopclosures, 1)
    key1 = outlier_loopclosures(m, 1) - 1;
    key2 = outlier_loopclosures(m, 2) - 1;
    pose1 = values.atPose3(key1);
    pose2 = values.atPose3(key2);
    plot([pose1.x, pose2.x], [pose1.y, pose2.y], 'LineWidth', 1, 'Color' ,[0.8 0.78 0.78],'LineStyle', ':')
    hold on
end
end

