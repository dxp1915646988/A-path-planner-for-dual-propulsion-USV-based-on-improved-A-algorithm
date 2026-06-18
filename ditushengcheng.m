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
ij_initial = [250, 1];
ij_destination = [171, 222];

% 检查起点和终点合法性
if max(ij_initial) > MM || gridMap(ij_initial(1), ij_initial(2)) == 1
    error('初始点不能设在障碍物上或超出范围');
end
if max(ij_destination) > MM || gridMap(ij_destination(1), ij_destination(2)) == 1
    error('目标点不能设在障碍物上或超出范围');
end

% 坐标转换
start_coord = [(ij_initial(2)-0.5)*Lgrid, (ij_initial(1)-0.5)*Lgrid];
goal_coord = [(ij_destination(2)-0.5)*Lgrid, (ij_destination(1)-0.5)*Lgrid];

% 人工势场参数
global K_att K_rep d0 force_threshold;
K_att = 0.01;        
K_rep = 3000.0;      
d0 = 500;            
force_threshold = 60;

% 方向向量
Dir = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];

% ================= 计算三种轨迹 ==================
fprintf('正在计算单独A*算法轨迹...\n');
[path_astar, path_coords_astar, g_astar] = astar_algorithm(gridMap, MM, ij_initial, ij_destination, false);
astar_inflection = count_inflection_points(path_coords_astar);

fprintf('正在计算人工势场结合A*算法轨迹...\n');
[path_potential, path_coords_potential, g_potential] = astar_algorithm(gridMap, MM, ij_initial, ij_destination, true);
potential_inflection = count_inflection_points(path_coords_potential);

fprintf('正在计算优化后轨迹...\n');
optimized_path_potential = optimize_path_with_bresenham(path_potential, gridMap, MM, ij_destination);
optimized_coords_potential = zeros(length(optimized_path_potential), 2);
for p = 1:length(optimized_path_potential)
    [i, j] = ind2sub([MM, MM], optimized_path_potential(p));
    optimized_coords_potential(p, :) = [(j-0.5)*Lgrid, (i-0.5)*Lgrid];
end
optimized_inflection = count_inflection_points(optimized_coords_potential);



%% ================= B样条平滑处理 ==================
fprintf('正在进行B样条平滑处理...\n');

% 1. 找拐点
[~, inflection_indices] = find_inflection_points(optimized_coords_potential);

% 2. 每个拐点生成4个控制点
control_points_list = cell(length(inflection_indices), 1);
for k = 1:length(inflection_indices)
    infl_idx = inflection_indices(k);
    control_points_list{k} = generate_single_inflection_controls(optimized_coords_potential, infl_idx, 2);
end

% 3. 合并所有控制点并排序
all_controls = vertcat(control_points_list{:});
[~, proj_indices] = project_points_to_path(all_controls, optimized_coords_potential);
[~, sorted_order] = sort(proj_indices);
sorted_controls = all_controls(sorted_order, :);

% 4. 每4个点为一组
num_groups = floor(size(sorted_controls, 1) / 4);
control_points_list = cell(num_groups, 1);
for g = 1:num_groups
    control_points_list{g} = sorted_controls((g-1)*4+1 : g*4, :);
end

% 5. 生成平滑路径
smooth_path_x = [];
smooth_path_y = [];
for g = 1:num_groups
    pts = control_points_list{g};
    if size(pts,1) == 4
        t = [0 0 0 0 1 1 1 1];
        sp = spmak(t, pts');
        s = linspace(0,1,100);
        curve_pts = fnval(sp, s);
        if isempty(smooth_path_x)
            smooth_path_x = curve_pts(1,:);
            smooth_path_y = curve_pts(2,:);
        else
            smooth_path_x = [smooth_path_x, curve_pts(1,2:end)];
            smooth_path_y = [smooth_path_y, curve_pts(2,2:end)];
        end
    end
end

smooth_path = [smooth_path_x', smooth_path_y'];  % 拼接后的平滑路径

% ================= 绘制轨迹结果 ==================
% 1. 单独A*
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
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');
plot(path_coords_astar(:,1)/Lgrid, path_coords_astar(:,2)/Lgrid,'r-','LineWidth',1.5,'DisplayName','A*轨迹');
legend('Location','best','FontName','华文中宋','FontSize',16);
title('单独A*算法的轨迹图','FontName','宋体','FontSize',14);
grid off;

% 2. 势场+A*
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
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');
plot(path_coords_potential(:,1)/Lgrid, path_coords_potential(:,2)/Lgrid,'-','Color',[0.6 0.3 0],'LineWidth',1.5,'DisplayName','势场+A*轨迹');
legend('Location','best','FontName','华文中宋','FontSize',16);
title('人工势场结合A*算法的轨迹图','FontName','宋体','FontSize',14);
grid off;

% 3. A* vs 势场+A*
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
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');
plot(path_coords_astar(:,1)/Lgrid, path_coords_astar(:,2)/Lgrid,'r-','LineWidth',1.5,'DisplayName','A*轨迹');
plot(path_coords_potential(:,1)/Lgrid, path_coords_potential(:,2)/Lgrid,'-','Color',[0.6 0.3 0],'LineWidth',1.5,'DisplayName','势场+A*轨迹');
legend('Location','best','FontName','华文中宋','FontSize',16);
title('A*与人工势场结合A*的轨迹对比图','FontName','宋体','FontSize',14);
grid off;

% 4. 优化后轨迹
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
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');
plot(optimized_coords_potential(:,1)/Lgrid, optimized_coords_potential(:,2)/Lgrid,'b-','LineWidth',1.5,'DisplayName','优化后轨迹');
legend('Location','best','FontName','华文中宋','FontSize',16);
title('Bresenham优化后的轨迹图','FontName','宋体','FontSize',14);
grid off;

% 5. 三种轨迹对比
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
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');
plot(path_coords_astar(:,1)/Lgrid, path_coords_astar(:,2)/Lgrid,'r-','LineWidth',1.5,'DisplayName','A*轨迹');
plot(path_coords_potential(:,1)/Lgrid, path_coords_potential(:,2)/Lgrid,'-','Color',[0.6 0.3 0],'LineWidth',1.5,'DisplayName','势场+A*轨迹');
plot(optimized_coords_potential(:,1)/Lgrid, optimized_coords_potential(:,2)/Lgrid,'b-','LineWidth',1.5,'DisplayName','优化后轨迹');
legend('Location','best','FontName','华文中宋','FontSize',16);
title('三种轨迹对比图','FontName','宋体','FontSize',14);
grid off;

%% 6. Bresenham优化 vs B样条平滑对比
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

% 起点、终点
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');

% Bresenham优化路径
plot(optimized_coords_potential(:,1)/Lgrid, optimized_coords_potential(:,2)/Lgrid,...
     'b-','LineWidth',1.5,'DisplayName','Bresenham优化');

% B样条平滑路径 + 首尾补线
plot(smooth_path(:,1)/Lgrid, smooth_path(:,2)/Lgrid,'m-','LineWidth',1.5,'DisplayName','B样条平滑');
plot([start_coord(1), smooth_path(1,1)]/Lgrid,...
     [start_coord(2), smooth_path(1,2)]/Lgrid,'m-','LineWidth',1.5, 'HandleVisibility', 'off');
plot([smooth_path(end,1), goal_coord(1)]/Lgrid,...
     [smooth_path(end,2), goal_coord(2)]/Lgrid,'m-','LineWidth',1.5, 'HandleVisibility', 'off');

legend('Location','best','FontName','华文中宋','FontSize',16);
title('直线优化与B样条平滑轨迹对比图','FontName','宋体','FontSize',14);
grid off;


%% 7. 四种路径对比 (A*、A*-APF、A*-APF-Bresenham、A*-APF-Bresenham-B样条)
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

% 起点、终点
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');

% A*路径
plot(path_coords_astar(:,1)/Lgrid, path_coords_astar(:,2)/Lgrid,'r-','LineWidth',1.5,'DisplayName','A*轨迹');

% A*-APF路径
plot(path_coords_potential(:,1)/Lgrid, path_coords_potential(:,2)/Lgrid,...
     '-','Color',[0.6 0.3 0],'LineWidth',1.5,'DisplayName','A*-APF轨迹');

% A*-APF-Bresenham
plot(optimized_coords_potential(:,1)/Lgrid, optimized_coords_potential(:,2)/Lgrid,...
     'b-','LineWidth',1.5,'DisplayName','A*-APF-Bresenham');

% A*-APF-Bresenham-B样条 + 首尾补线
plot(smooth_path(:,1)/Lgrid, smooth_path(:,2)/Lgrid,'m-','LineWidth',1.5,'DisplayName','A*-APF-Bresenham-B样条');
plot([start_coord(1), smooth_path(1,1)]/Lgrid,...
     [start_coord(2), smooth_path(1,2)]/Lgrid,'m-','LineWidth',1.5, 'HandleVisibility', 'off');
plot([smooth_path(end,1), goal_coord(1)]/Lgrid,...
     [smooth_path(end,2), goal_coord(2)]/Lgrid,'m-','LineWidth',1.5, 'HandleVisibility', 'off');

legend('Location','best','FontName','华文中宋','FontSize',16);
title('四种路径对比图','FontName','宋体','FontSize',14);
grid off;
full_path_original = [
    start_coord;                   % 原始比例的起点
    smooth_path;                   % 原始比例的B样条路径
    goal_coord                     % 原始比例的目标点
];
save('full_path_original.mat', 'full_path_original');



% ================= 路径信息打印 ==================
fprintf('\n单独A*算法 - 总长度: %.2f m\n', g_astar * Lgrid);
fprintf('单独A*算法 - 路径拐点数: %d\n', astar_inflection);
fprintf('\n人工势场结合A*算法 - 总长度: %.2f m\n', g_potential * Lgrid);
fprintf('人工势场结合A*算法 - 路径拐点数: %d\n', potential_inflection);
optimized_length = 0;
for p = 2:length(optimized_path_potential)
    dx = optimized_coords_potential(p, 1) - optimized_coords_potential(p-1, 1);
    dy = optimized_coords_potential(p, 2) - optimized_coords_potential(p-1, 2);
    optimized_length = optimized_length + sqrt(dx^2 + dy^2);
end
fprintf('\n优化后路径 - 总长度: %.2f m\n', optimized_length);
fprintf('优化后路径 - 路径拐点数: %d\n', optimized_inflection);
% B样条平滑路径长度计算
bspline_length = 0;
for p = 2:length(smooth_path)
    dx = smooth_path(p,1) - smooth_path(p-1,1);
    dy = smooth_path(p,2) - smooth_path(p-1,2);
    bspline_length = bspline_length + sqrt(dx^2 + dy^2);
end
% 补上首尾段
dx_start = smooth_path(1,1) - start_coord(1);
dy_start = smooth_path(1,2) - start_coord(2);
dx_end   = goal_coord(1) - smooth_path(end,1);
dy_end   = goal_coord(2) - smooth_path(end,2);
bspline_length = bspline_length + sqrt(dx_start^2 + dy_start^2) + sqrt(dx_end^2 + dy_end^2);

fprintf('\nB样条平滑路径 - 总长度: %.2f m\n', bspline_length);
fprintf('B样条平滑路径 - 路径拐点数: %d\n', 0); % B样条已平滑，可视为无拐点

%% 4. 路径规划辅助函数（保持原有算法逻辑）
function [path, path_coords, g_cost_total] = astar_algorithm(G, MM, start, goal, use_potential)
    global Dir K_att K_rep d0 Lgrid;

    % 起点和终点索引
    start_node = sub2ind([MM, MM], start(1), start(2));
    goal_node = sub2ind([MM, MM], goal(1), goal(2));
    if start_node == goal_node
        error('起点和终点相同！');
    end

    % 初始化参数
    open_list = [];
    closed_list = zeros(MM, MM);
    parent = zeros(MM*MM, 1);
    g_cost = inf(MM*MM, 1);
    h_cost = zeros(MM*MM, 1);
    f_cost = inf(MM*MM, 1);

    % 计算启发式h_cost
    for i = 1:MM
        for j = 1:MM
            if G(i,j) ~= 1
                node_idx = sub2ind([MM, MM], i, j);
                if use_potential
                    % 势场结合A*：h = 势场势能 + 曼哈顿距离
                    h_cost(node_idx) = potential_field([i,j], goal, G) + manhattan_distance([i,j], goal);
                else
                    % 单独A*：h = 曼哈顿距离
                    h_cost(node_idx) = manhattan_distance([i,j], goal);                    
                end
            end
        end
    end

    % 起点初始化
    g_cost(start_node) = 0;
    f_cost(start_node) = h_cost(start_node);
    open_list = [start_node, f_cost(start_node)];

    % A*主循环
    found_path = false;
    iter_count = 0;
    while ~isempty(open_list)
        iter_count = iter_count + 1;
        [~, idx] = min(open_list(:, 2));
        current_node = open_list(idx, 1);
        open_list(idx, :) = [];

        if current_node == goal_node
            found_path = true;
            break;
        end

        [i, j] = ind2sub([MM, MM], current_node);
        closed_list(i, j) = 1;

        % 遍历邻域
        for dir = 1:8
            ni = i + Dir(dir, 1);
            nj = j + Dir(dir, 2);
            if ni < 1 || ni > MM || nj < 1 || nj > MM || G(ni, nj) == 1 || closed_list(ni, nj) == 1
                continue;
            end


            % 2. 新增：对角移动时，检查是否跨越障碍（关键修复）
            diagonal_move = (abs(Dir(dir, 1)) + abs(Dir(dir, 2)) == 2); % 判断是否为对角移动
            if diagonal_move
                % 检查对角移动的两个相邻栅格（如从(i,j)到(i+1,j+1)，需检查(i+1,j)和(i,j+1)）
                grid1 = G(i + Dir(dir, 1), j); % 同行邻栅格
                grid2 = G(i, j + Dir(dir, 2)); % 同列邻栅格
                if grid1 == 1 || grid2 == 1 % 任一为障碍，禁止对角移动
                    continue;
                end
            end

            % 移动代价（对角为√2倍栅格尺寸）
            movement_cost = 1.0;
            if abs(Dir(dir, 1)) + abs(Dir(dir, 2)) == 2
                movement_cost = 1.414;  % √2≈1.414
            end

            neighbor_node = sub2ind([MM, MM], ni, nj);
            new_g_cost = g_cost(current_node) + movement_cost;

            if ~ismember(neighbor_node, open_list(:,1)) || new_g_cost < g_cost(neighbor_node)
                parent(neighbor_node) = current_node;
                g_cost(neighbor_node) = new_g_cost;
                f_cost(neighbor_node) = new_g_cost + h_cost(neighbor_node);
                if ~ismember(neighbor_node, open_list(:,1))
                    open_list = [open_list; neighbor_node, f_cost(neighbor_node)];
                end
            end
        end

        if iter_count > 10000
            warning('达到最大迭代次数');
            break;
        end
    end

    % 回溯路径
    if ~found_path
        error('未找到路径');
    end
    path = [];
    current = goal_node;
    while current ~= 0
        path = [current; path];
        current = parent(current);
    end

    % 转换为实际米坐标
    path_coords = zeros(length(path), 2);
    for p = 1:length(path)
        [i, j] = ind2sub([MM, MM], path(p));
        path_coords(p, :) = [(j-0.5)*Lgrid, (i-0.5)*Lgrid];
    end
    g_cost_total = g_cost(goal_node) * Lgrid;  % 转换为实际米数
end

function optimized_path = optimize_path_with_bresenham(path, G, MM, goal)
    if length(path) <= 2
        optimized_path = path;
        return;
    end
    optimized_path = [path(1)];
    current_idx = 1;
    while current_idx < length(path)
        found = false;
        for next_idx = length(path):-1:current_idx+1
            [x0, y0] = ind2sub([MM, MM], path(current_idx));
            [x1, y1] = ind2sub([MM, MM], path(next_idx));
            if is_line_safe(x0, y0, x1, y1, G, MM, goal)
                optimized_path = [optimized_path; path(next_idx)];
                current_idx = next_idx;
                found = true;
                break;
            end
        end
        if ~found
            current_idx = current_idx + 1;
            optimized_path = [optimized_path; path(current_idx)];
        end
    end
end


function safe = is_line_safe(x0, y0, x1, y1, G, MM, goal)
    global force_threshold;
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    sx = 1; if x1 < x0, sx = -1; end
    sy = 1; if y1 < y0, sy = -1; end
    err = dx - dy;
    x = x0; y = y0;

    while true
        % 1. 原栅格障碍检测（不变）
        if x < 1 || x > MM || y < 1 || y > MM || G(x, y) == 1
            safe = false;
            return;
        end
        % 2. 新增：检查当前直线是否与相邻障碍栅格的边缘相交（关键修复）
        % 遍历当前栅格的4个相邻栅格（上、下、左、右）
        neighbors = [x+1,y; x-1,y; x,y+1; x,y-1];
        for k = 1:size(neighbors,1)
            nx = neighbors(k,1); ny = neighbors(k,2);
            if nx >=1 && nx <= MM && ny >=1 && ny <= MM && G(nx, ny) == 1
                % 若相邻栅格是障碍，判断直线是否穿过当前栅格与障碍栅格的边界
                if (x == nx && (y0 < ny && y1 > ny || y0 > ny && y1 < ny)) || ...
                   (y == ny && (x0 < nx && x1 > nx || x0 > nx && x1 < nx))
                    safe = false;
                    return;
                end
            end
        end
        % 3. 原合力阈值检测（不变）
        if calculate_resultant_force([x, y], goal, G, MM) > force_threshold
            safe = false;
            return;
        end
        if x == x1 && y == y1, break; end
        e2 = 2 * err;
        if e2 > -dy, err = err - dy; x = x + sx; end
        if e2 < dx, err = err + dx; y = y + sy; end
    end
    safe = true;
end

function force = calculate_resultant_force(point, goal, G, MM)
    global K_att K_rep d0 Lgrid;
    % 引力（转换为实际米单位计算）
    dx_att = (goal(1) - point(1)) * Lgrid;
    dy_att = (goal(2) - point(2)) * Lgrid;
    dist_att = sqrt(dx_att^2 + dy_att^2) + eps;  % 防止除零
    F_att_x = K_att * dx_att / dist_att;
    F_att_y = K_att * dy_att / dist_att;

    %% 斥力（仅计算障碍附近区域）
    F_rep_x = 0; F_rep_y = 0;
    obstacle_dist_threshold = 100;  % 5米内障碍计算斥力
    for i = 1:MM
        for j = 1:MM
            if G(i,j) == 1
                dx_rep = (point(1) - i) * Lgrid;
                dy_rep = (point(2) - j) * Lgrid;
                dist_rep = sqrt(dx_rep^2 + dy_rep^2);
                if dist_rep > obstacle_dist_threshold || dist_rep < eps
                    continue;
                end
                if dist_rep <= d0
                    F_rep_mag = K_rep * (1/dist_rep - 1/d0) * (1/dist_rep^2);
                    F_rep_x = F_rep_x + F_rep_mag * (dx_rep / dist_rep);
                    F_rep_y = F_rep_y + F_rep_mag * (dy_rep / dist_rep);
                end
            end
        end
    end
    force = sqrt((F_att_x + F_rep_x)^2 + (F_att_y + F_rep_y)^2);
end

function potential = potential_field(node, goal, G)
    global K_att K_rep d0 MM Lgrid;
    % 引力势能（实际米单位）
    dx_att = abs(node(1) - goal(1)) * Lgrid;
    dy_att = abs(node(2) - goal(2)) * Lgrid;
    dist_att = sqrt(dx_att^2 + dy_att^2);
    U_att = 0.5 * K_att * dist_att^2;

    % 斥力势能
    U_rep = 0;
    obstacle_dist_threshold = 100;  % 5米内障碍
    for i = 1:MM
        for j = 1:MM
            if G(i,j) == 1
                dx_rep = abs(node(1) - i) * Lgrid;
                dy_rep = abs(node(2) - j) * Lgrid;
                dist_rep = sqrt(dx_rep^2 + dy_rep^2);
                if dist_rep > obstacle_dist_threshold
                    continue;
                end
                if dist_rep <= d0 && dist_rep > eps
                    U_rep = U_rep + 0.5 * K_rep * (1/dist_rep - 1/d0)^2;
                end
            end
        end
    end
    potential = U_att + U_rep;
end

function dist = manhattan_distance(node, goal)
    % 曼哈顿距离（栅格数）
    dist = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function dist = euclidean_distance(node, goal)
    % 欧式距离（栅格数）
    dx = node(1) - goal(1);
    dy = node(2) - goal(2);
    dist = sqrt(dx^2 + dy^2);
end

function count = count_inflection_points(path_coords)
    n = size(path_coords, 1);
    count = 0;

    % 少于3个点无拐点
    if n < 3
        return;
    end

    % 检测连续三点的方向变化
    for i = 3:n
        p_prev = path_coords(i-2, :);
        p_curr = path_coords(i-1, :);
        p_next = path_coords(i, :);

        v1 = p_curr - p_prev;  % 前向向量
        v2 = p_next - p_curr;  % 后向向量

        % 叉积判断方向变化（非零即拐点）
        cross_product = v1(1)*v2(2) - v1(2)*v2(1);
        if abs(cross_product) > 1e-6
            count = count + 1;
        end
    end
end







function [count, indices] = find_inflection_points(path_coords)
    n = size(path_coords, 1);
    count = 0;
    indices = [];
    if n < 3, return; end
    for i = 3:n
        p_prev = path_coords(i-2, :);
        p_curr = path_coords(i-1, :);
        p_next = path_coords(i, :);
        v1 = p_curr - p_prev;  % 前向向量
        v2 = p_next - p_curr;  % 后向向量
        cross_product = v1(1)*v2(2) - v1(2)*v2(1);  % 2D叉积（判断方向变化）
        if abs(cross_product) > 1e-6  % 非共线即拐点
            count = count + 1;
            indices = [indices; i-1];  % 拐点位置为中间点
        end
    end
end

function controls = generate_single_inflection_controls(original_path, infl_idx, half_range)
    n = size(original_path, 1);
    dist_cum = zeros(n, 1);
    for i = 2:n
        dx = original_path(i,1) - original_path(i-1,1);
        dy = original_path(i,2) - original_path(i-1,2);
        dist_cum(i) = dist_cum(i-1) + sqrt(dx^2 + dy^2);
    end
    infl_dist = dist_cum(infl_idx);
    start_dist = infl_dist - half_range;
    end_dist = infl_dist + half_range;
    
    start_idx = infl_idx;
    while start_idx > 1 && dist_cum(start_idx) > start_dist
        start_idx = start_idx - 1;
    end
    end_idx = infl_idx;
    while end_idx < n && dist_cum(end_idx) < end_dist
        end_idx = end_idx + 1;
    end
    
    controls = zeros(4, 2);
    for i = 1:4
        t = (i-1)/3;
        target_dist = start_dist + t*(end_dist - start_dist);
        seg_idx = find(dist_cum >= target_dist, 1);
        if isempty(seg_idx), seg_idx = n; end
        if seg_idx == 1, seg_idx = 2; end
        ratio = (target_dist - dist_cum(seg_idx-1)) / (dist_cum(seg_idx) - dist_cum(seg_idx-1));
        controls(i,:) = original_path(seg_idx-1,:) + ratio*(original_path(seg_idx,:) - original_path(seg_idx-1,:));
    end
end

function [proj_points, proj_indices] = project_points_to_path(points, path)
    n = size(points, 1);
    proj_points = zeros(n, 2);
    proj_indices = zeros(n, 1);
    path_len = size(path, 1);
    
    for i = 1:n
        min_dist = Inf;
        best_idx = 1;
        best_proj = path(1,:);
        
        for j = 2:path_len
            p1 = path(j-1,:);
            p2 = path(j,:);
            t = max(0, min(1, dot(points(i,:)-p1, p2-p1)/norm(p2-p1)^2));
            proj = p1 + t*(p2-p1);
            dist = norm(points(i,:)-proj);
            
            if dist < min_dist
                min_dist = dist;
                best_proj = proj;
                best_idx = j-1 + t;
            end
        end
        
        proj_points(i,:) = best_proj;
        proj_indices(i) = best_idx;
    end
end