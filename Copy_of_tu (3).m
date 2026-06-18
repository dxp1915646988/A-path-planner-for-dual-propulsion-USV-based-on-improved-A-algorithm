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
ij_initial = [250, 1];    %起点250行1列
ij_destination = [96, 215];

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

% ================= RRT* 算法实现 ==================

% RRT*参数设置
maxIter = 10000;         % 最大迭代次数
stepSize = 2;           % 步长大小5 (栅格数)
goalSampleRate = 0.15;   % 目标点采样概率
radius = 10;            % 邻域半径 (栅格数)

% 初始化树结构
tree.nodes = start_coord;  % 节点坐标 (x, y)
tree.parent = 0;          % 父节点索引，初始为空[]
tree.cost = 0;             % 从起点到该节点的成本

% 记录路径是否找到
pathFound = false;
finalIndex = -1;

% RRT*主循环
for iter = 1:maxIter
    % 随机采样一个点
    if rand < goalSampleRate
        % 以一定概率直接采样目标点
        randPoint = goal_coord;
    else
        % 随机采样自由空间点
        randPoint = [rand * gridSize * cellSize, rand * gridSize * cellSize];
    end
    
    % 找到最近的节点
    [nearestIdx, nearestDist] = findNearestNode(tree.nodes, randPoint);
    
    % 从最近节点向采样点扩展
    newPoint = steer(tree.nodes(nearestIdx,:), randPoint, stepSize * cellSize);
    
    % 将坐标转换为栅格索引进行碰撞检测
    ij_new = [round(newPoint(2)/cellSize + 0.5), round(newPoint(1)/cellSize + 0.5)];
    
    % 检查新节点是否合法（不在障碍物中且在地图范围内）
    if ij_new(1) >= 1 && ij_new(1) <= gridSize && ij_new(2) >= 1 && ij_new(2) <= gridSize && ...
            gridMap(ij_new(1), ij_new(2)) == 0 && ...
            isCollisionFree(tree.nodes(nearestIdx,:), newPoint, gridMap, cellSize, gridSize)
        
        % 找到新节点附近的节点
        nearIndices = findNearNodes(tree.nodes, newPoint, radius * cellSize);
        
        % 初始化新节点的父节点和成本
        minCost = tree.cost(nearestIdx) + distance(tree.nodes(nearestIdx,:), newPoint);
        bestParentIdx = nearestIdx;
        
        % 检查附近节点，寻找更优父节点
        for i = 1:length(nearIndices)
            nearIdx = nearIndices(i);
            % 确保索引有效
            if nearIdx < 1 || nearIdx > length(tree.cost)
                continue;
            end
            
            if isCollisionFree(tree.nodes(nearIdx,:), newPoint, gridMap, cellSize, gridSize)
                newCost = tree.cost(nearIdx) + distance(tree.nodes(nearIdx,:), newPoint);
                if newCost < minCost
                    minCost = newCost;
                    bestParentIdx = nearIdx;
                end
            end
        end
        
        % 确保最佳父节点索引有效
        if bestParentIdx < 1 || bestParentIdx > length(tree.nodes)
            warning('选择了无效的父节点索引，跳过该节点');
            continue;
        end
        
        % 添加新节点到树中
        tree.nodes = [tree.nodes; newPoint];
        tree.parent = [tree.parent; bestParentIdx];
        tree.cost = [tree.cost; minCost];
        
        % 重新连接附近节点，可能找到更优路径
        for i = 1:length(nearIndices)
            nearIdx = nearIndices(i);
            % 确保索引有效
            if nearIdx < 1 || nearIdx > length(tree.cost) || length(tree.cost) < 1
                continue;
            end
            
            if tree.cost(end) + distance(newPoint, tree.nodes(nearIdx,:)) < tree.cost(nearIdx) && ...
                    isCollisionFree(newPoint, tree.nodes(nearIdx,:), gridMap, cellSize, gridSize)
                tree.parent(nearIdx) = length(tree.nodes);
                tree.cost(nearIdx) = tree.cost(end) + distance(newPoint, tree.nodes(nearIdx,:));
            end
        end
        
        % 检查是否到达目标附近
        if distance(newPoint, goal_coord) < stepSize * cellSize
            % 将目标点添加到树中
            tree.nodes = [tree.nodes; goal_coord];
            % 确保父节点索引有效（应为当前最后一个节点）
            validParentIdx = length(tree.nodes) - 1;
            if validParentIdx < 1
                validParentIdx = 1;
            end
            tree.parent = [tree.parent; validParentIdx];
            tree.cost = [tree.cost; tree.cost(end) + distance(newPoint, goal_coord)];
            pathFound = true;
            finalIndex = length(tree.nodes);
            break;
        end
    end
    
    % 每100次迭代显示进度
    if mod(iter, 100) == 0
        fprintf('迭代次数: %d/%d\n', iter, maxIter);
    end
end

% 如果找到路径，回溯构建路径
if pathFound && finalIndex > 0 && finalIndex <= length(tree.nodes)
    % 回溯路径
    pathIndices = [];
    currentIdx = finalIndex;
    backtrackCount = 0;
    maxBacktrack = length(tree.parent) * 2;  % 设置最大回溯次数防止无限循环
    
    while true
        % 检查当前索引是否有效
        if currentIdx < 1 || currentIdx > length(tree.parent)
            warning('回溯过程中遇到无效索引，路径可能不完整');
            break;
        end
        
        % 添加当前索引到路径
        pathIndices = [currentIdx, pathIndices];
        
        % 如果到达起点，退出循环
        if currentIdx == 1
            break;
        end
        
        % 获取父节点索引
        nextIdx = tree.parent(currentIdx);
        
        % 检查是否陷入循环
        if backtrackCount >= maxBacktrack
            warning('回溯次数超过最大值，可能存在循环');
            break;
        end
        
        % 检查是否父节点索引不变（防止循环）
        if nextIdx == currentIdx
            warning('检测到自循环的父节点引用');
            break;
        end
        
        currentIdx = nextIdx;
        backtrackCount = backtrackCount + 1;
    end
    
    % 验证所有路径索引是否有效
    validIndices = pathIndices(pathIndices >= 1 & pathIndices <= size(tree.nodes, 1));
    if length(validIndices) ~= length(pathIndices)
        warning('路径中包含无效索引，已自动过滤');
    end
    
    % 提取路径坐标
    if ~isempty(validIndices)
        path_coords_rrtstar = tree.nodes(validIndices, :);
        
        % 显示RRT*结果
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
        
        % 绘制起点、终点和路径
        plot(start_coord(1)/cellSize, start_coord(2)/cellSize, 'go','MarkerSize',5,'MarkerFaceColor','g');
        plot(goal_coord(1)/cellSize, goal_coord(2)/cellSize, 'mo','MarkerSize',5,'MarkerFaceColor','m');
        plot(path_coords_rrtstar(:,1)/cellSize, path_coords_rrtstar(:,2)/cellSize,'r-','LineWidth',1.5,'DisplayName','RRT*轨迹');
        legend('起点','终点','RRT*轨迹','Location','best','FontName','华文中宋','FontSize',16);
        title('RRT*算法的轨迹图','FontName','宋体','FontSize',14);
        grid off;
        % save("path_coords_rrtstar.mat", "path_coords_rrtstar");
    else
        warning('无法构建有效路径');
    end
else
    warning('在最大迭代次数内未找到路径！');
end

% ================ RRT*辅助函数 ================
function [nearestIdx, nearestDist] = findNearestNode(nodes, point)
    % 找到树中距离给定点最近的节点
    distances = sqrt(sum((nodes - repmat(point, size(nodes,1), 1)).^2, 2));
    [nearestDist, nearestIdx] = min(distances);
end

function newPoint = steer(fromPoint, toPoint, stepSize)
    % 从fromPoint向toPoint方向移动stepSize距离
    dirVec = toPoint - fromPoint;
    dist = norm(dirVec);
    if dist <= stepSize || dist < eps
        newPoint = toPoint;
    else
        newPoint = fromPoint + (dirVec / dist) * stepSize;
    end
end

function d = distance(point1, point2)
    % 计算两点之间的欧氏距离
    d = norm(point1 - point2);
end

function nearIndices = findNearNodes(nodes, point, radius)
    % 找到树中在给定点半径范围内的所有节点
    if size(nodes, 1) == 0
        nearIndices = [];
        return;
    end
    distances = sqrt(sum((nodes - repmat(point, size(nodes,1), 1)).^2, 2));
    nearIndices = find(distances <= radius);
end

function collisionFree = isCollisionFree(point1, point2, gridMap, cellSize, gridSize)
    % 检查两点之间的线段是否与障碍物碰撞
    
    % 将坐标转换为栅格索引
    x1 = point1(1); y1 = point1(2);
    x2 = point2(1); y2 = point2(2);
    
    % 栅格索引（行，列）- 注意行对应y，列对应x
    i1 = round(y1 / cellSize + 0.5);
    j1 = round(x1 / cellSize + 0.5);
    i2 = round(y2 / cellSize + 0.5);
    j2 = round(x2 / cellSize + 0.5);
    
    % 使用Bresenham算法检查线段经过的所有栅格
    [i, j] = bresenham(i1, j1, i2, j2);
    
    % 检查所有经过的栅格是否为自由空间
    collisionFree = true;
    for k = 1:length(i)
        if i(k) < 1 || i(k) > gridSize || j(k) < 1 || j(k) > gridSize || gridMap(i(k), j(k)) == 1
            collisionFree = false;
            return;
        end
    end
end

function [x, y] = bresenham(x0, y0, x1, y1)
    % Bresenham算法：生成两点之间的栅格坐标
    % 将输入坐标转换为整数栅格索引
    x0 = round(x0);
    y0 = round(y0);
    x1 = round(x1);
    y1 = round(y1);
    
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    sx = sign(x1 - x0);
    sy = sign(y1 - y0);
    err = dx - dy;
    
    % 初始化坐标数组，存储起点
    x = x0;
    y = y0;
    
    % 当前点
    current_x = x0;
    current_y = y0;
    
    % 循环直到到达终点
    while current_x ~= x1 || current_y ~= y1
        e2 = 2 * err;
        
        % x方向移动
        if e2 > -dy
            err = err - dy;
            current_x = current_x + sx;
        end
        
        % y方向移动
        if e2 < dx
            err = err + dx;
            current_y = current_y + sy;
        end
        
        % 将新点添加到数组
        x = [x; current_x];
        y = [y; current_y];
    end
    
    % 去除可能的连续重复点（保留顺序）
    [~, idx] = unique([x y], 'rows', 'stable');
    x = x(idx);
    y = y(idx);
end
