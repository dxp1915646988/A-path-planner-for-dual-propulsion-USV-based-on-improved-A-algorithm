clc; clear; close all;
% 加载保存的栅格地图数据
load('gridMap.mat', 'gridMap');

% 设置参数（与原代码保持一致）
gridSize = size(gridMap, 1);  % 从地图数据中获取栅格数量
cellSize = 4;                 % 每个栅格边长 (m)

% 绘制栅格地图
figure('Color','w','Units','normalized','Position',[0 0 1 1]); 
imshow(gridMap, 'InitialMagnification', 'fit');  % 显示栅格图
colormap([1 1 1; 0 0 0]);      % 白色 = 可通行, 黑色 = 障碍
axis equal; 
axis on;
set(gca, 'YDir', 'reverse');
xticks(0:gridSize/10:gridSize);
xticklabels(0:gridSize*cellSize/10:gridSize*cellSize);
yticks(0:gridSize/10:gridSize);
yticklabels(fliplr(0:gridSize*cellSize/10:gridSize*cellSize));
set(gca, 'FontName', 'Times New Roman', 'FontSize', 16);
box off;
set(gca,'XAxisLocation','bottom','YAxisLocation','left');
xlabel("$x$/m", "interpreter", "latex", 'FontSize', 18, 'FontName', 'Times New Roman');
ylabel('$y$/m', "interpreter", "latex", 'FontSize', 18, 'FontName', 'Times New Roman');
axis([0 gridSize 0 gridSize-0.1]);
grid off;

% ================= 路径规划部分 ==================
global MM Dir Lgrid;
Lgrid = cellSize;
MM = gridSize;

% 起点和终点栅格索引 (行, 列)
ij_initial = [250-1, 1+1];    %起点250行1列
ij_destination = [96, 215];




% 加载三条路径数据
load('path_coords_pso.mat', 'path_coords_pso');
load('path_coords_rrtstar.mat', 'path_coords_rrtstar');
load('full_path.mat', 'full_path');

% 将栅格索引转换为坐标（行对应y，列对应x）
start_coord = [ij_initial(2)*Lgrid, ij_initial(1)*Lgrid];  % 起点坐标(列,行) -> (x,y)
goal_coord = [ij_destination(2)*Lgrid, ij_destination(1)*Lgrid];  % 终点坐标

% 绘制路径对比图
figure('Color','w','Units','normalized','Position',[0 0 1 1]); 
imshow(gridMap, 'InitialMagnification', 'fit');
colormap([1 1 1; 0 0 0]);
axis equal; axis on; set(gca,'YDir','reverse');
xticks(0:gridSize/10:gridSize);
xticklabels(0:gridSize*cellSize/10:gridSize*cellSize);
yticks(0:gridSize/10:gridSize);
yticklabels(fliplr(0:gridSize*cellSize/10:gridSize*cellSize));
set(gca,'FontName','Times New Roman','FontSize',16);
box off;
set(gca,'XAxisLocation','bottom','YAxisLocation','left');
xlabel("$x$/m","interpreter","latex",'FontSize',18,'FontName','Times New Roman');
ylabel('$y$/m',"interpreter","latex",'FontSize',18,'FontName','Times New Roman');
axis([0 gridSize 0 gridSize-0.1]); hold on;

% 绘制起点和终点
% plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',7,'MarkerFaceColor','g','DisplayName','起始点');
% plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',7,'MarkerFaceColor','m','DisplayName','目标点');

% 绘制起点和终点
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'o', ...
    'MarkerSize',7, 'MarkerFaceColor',[0.6 0.1 0.9], ...  % 紫色
    'MarkerEdgeColor',[0.6 0.1 0.9], 'DisplayName','起始点');

plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'o', ...
    'MarkerSize',7, 'MarkerFaceColor',[0.2 0.55 0.34], ...  % 深绿
    'MarkerEdgeColor',[0.2 0.55 0.34], 'DisplayName','目标点');


% 绘制三条路径
plot((path_coords_pso(:,1))/Lgrid, (path_coords_pso(:,2))/Lgrid, 'b-','LineWidth',1.5,'DisplayName','PSO算法路径');
plot(path_coords_rrtstar(:,1)/Lgrid, path_coords_rrtstar(:,2)/Lgrid, 'Color',[0.6 0.3 0],'LineWidth',1.5,'DisplayName','RRT*算法路径');
plot(full_path(:,1)/Lgrid, full_path(:,2)/Lgrid, 'r-','LineWidth',1.5,'DisplayName','改进A*算法路径');

% 添加图例和标题
legend('Location','best','FontName','华文中宋','FontSize',16);
title('三种路径规划算法轨迹对比图','FontName','宋体','FontSize',18);
grid off;

% 计算各路径长度
% 1. PSO路径长度
pso_length = 0;
for i = 1:size(path_coords_pso, 1)-1
    % 计算相邻两点之间的欧氏距离
    dx = path_coords_pso(i+1, 1) - path_coords_pso(i, 1);
    dy = path_coords_pso(i+1, 2) - path_coords_pso(i, 2);
    pso_length = pso_length + sqrt(dx^2 + dy^2);
end

% 2. RRT*路径长度
rrtstar_length = 0;
for i = 1:size(path_coords_rrtstar, 1)-1
    dx = path_coords_rrtstar(i+1, 1) - path_coords_rrtstar(i, 1);
    dy = path_coords_rrtstar(i+1, 2) - path_coords_rrtstar(i, 2);
    rrtstar_length = rrtstar_length + sqrt(dx^2 + dy^2);
end

% 3. 平滑路径长度
smooth_length = 0;
for i = 1:size(full_path, 1)-1
    dx = full_path(i+1, 1) - full_path(i, 1);
    dy = full_path(i+1, 2) - full_path(i, 2);
    smooth_length = smooth_length + sqrt(dx^2 + dy^2);
end

% 显示路径长度结果
fprintf('PSO路径长度: %.2f 米\n', pso_length);
fprintf('RRT*路径长度: %.2f 米\n', rrtstar_length);
fprintf('平滑路径长度: %.2f 米\n', smooth_length);


% 计算三条路径的拐点数
pso_turns     = count_turns(path_coords_pso);
rrtstar_turns = count_turns(path_coords_rrtstar);
smooth_turns  = count_turns(full_path);

% 显示拐点结果
fprintf('PSO路径拐点数: %d\n', pso_turns);
fprintf('RRT*路径拐点数: %d\n', rrtstar_turns);
fprintf('平滑路径拐点数: %d\n', smooth_turns);



% ================== 计算拐点数量 ==================

% 定义计算拐点函数
function num_turns = count_turns(path)
    num_turns = 0;
    for i = 2:size(path,1)-1
        v1 = path(i,:) - path(i-1,:);   % 前一段向量
        v2 = path(i+1,:) - path(i,:);   % 后一段向量
        % 判断夹角是否变化（用叉积判断方向变化，避免浮点误差）
        if norm(v1) > 1e-6 && norm(v2) > 1e-6
            cos_theta = dot(v1,v2)/(norm(v1)*norm(v2));
            if cos_theta < 0.999   % 小于阈值说明有拐点（阈值可调）
                num_turns = num_turns + 1;
            end
        end
    end
end