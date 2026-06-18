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


% ================= PSO路径规划参数设置 ==================
% PSO参数
num_particles = 150;    % 粒子数量150
max_iter = 120;        % 最大迭代次数100
c1 = 1.5;              % 个体学习因子1.5
c2 = 1.5;              % 社会学习因子1.5
w_start = 0.9;         % 初始惯性权重0.9
w_end = 0.4;           % 结束惯性权重0.4
num_points = 10;       % 每条路径的中间点数量10

% 获取起点和终点坐标
start_x = start_coord(1);
start_y = start_coord(2);
goal_x = goal_coord(1);
goal_y = goal_coord(2);

% 初始化粒子群
particles = struct('position', [], 'velocity', [], 'best_position', [], 'best_fitness', Inf);
for i = 1:num_particles
    % 初始化路径点（起点到终点的随机点）
    x = linspace(start_x, goal_x, num_points+2);
    y = linspace(start_y, goal_y, num_points+2);
    
    % 添加随机扰动
    for j = 2:num_points+1
        x(j) = x(j) + (rand-0.5)*20;  % 横向扰动
        y(j) = y(j) + (rand-0.5)*20;  % 纵向扰动
        
        % 确保在地图范围内
        x(j) = max(x(j), Lgrid/2);
        x(j) = min(x(j), MM*Lgrid - Lgrid/2);
        y(j) = max(y(j), Lgrid/2);
        y(j) = min(y(j), MM*Lgrid - Lgrid/2);
    end
    
    particles(i).position = [x; y];
    particles(i).velocity = zeros(2, num_points+2);
    
    % 计算初始适应度
    fitness = calculate_fitness(particles(i).position, gridMap, Lgrid, MM);
    particles(i).best_position = particles(i).position;
    particles(i).best_fitness = fitness;
end

% 初始化全局最优
global_best_fitness = Inf;
global_best_position = [];
for i = 1:num_particles
    if particles(i).best_fitness < global_best_fitness
        global_best_fitness = particles(i).best_fitness;
        global_best_position = particles(i).best_position;
    end
end

% ================= PSO迭代优化 ==================
fitness_history = zeros(1, max_iter);

for iter = 1:max_iter
    % 动态调整惯性权重
    w = w_start - (w_start - w_end) * (iter / max_iter);
    
    for i = 1:num_particles
        % 更新速度
        r1 = rand;
        r2 = rand;
        particles(i).velocity = w * particles(i).velocity ...
            + c1 * r1 * (particles(i).best_position - particles(i).position) ...
            + c2 * r2 * (global_best_position - particles(i).position);
        
        % 更新位置
        particles(i).position = particles(i).position + particles(i).velocity;
        
        % 边界处理
        for j = 1:num_points+2
            particles(i).position(1,j) = max(particles(i).position(1,j), Lgrid/2);
            particles(i).position(1,j) = min(particles(i).position(1,j), MM*Lgrid - Lgrid/2);
            particles(i).position(2,j) = max(particles(i).position(2,j), Lgrid/2);
            particles(i).position(2,j) = min(particles(i).position(2,j), MM*Lgrid - Lgrid/2);
        end
        
        % 固定起点和终点
        particles(i).position(1,1) = start_x;
        particles(i).position(2,1) = start_y;
        particles(i).position(1,end) = goal_x;
        particles(i).position(2,end) = goal_y;
        
        % 计算适应度
        current_fitness = calculate_fitness(particles(i).position, gridMap, Lgrid, MM);
        
        % 更新个体最优
        if current_fitness < particles(i).best_fitness
            particles(i).best_position = particles(i).position;
            particles(i).best_fitness = current_fitness;
            
            % 更新全局最优
            if current_fitness < global_best_fitness
                global_best_fitness = current_fitness;
                global_best_position = particles(i).position;
            end
        end
    end
    
    % 记录历史适应度
    fitness_history(iter) = global_best_fitness;
    
    % 显示迭代信息
    fprintf('迭代次数: %d, 最优适应度: %.4f\n', iter, global_best_fitness);
end

% ================= 绘制结果 ==================
% 提取最优路径
path_x = global_best_position(1,:);
path_y = global_best_position(2,:);
path_coords_pso = [path_x; path_y]';

% 绘制路径图
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
plot(start_coord(1)/Lgrid, start_coord(2)/Lgrid, 'go','MarkerSize',5,'MarkerFaceColor','g');
plot(goal_coord(1)/Lgrid, goal_coord(2)/Lgrid, 'mo','MarkerSize',5,'MarkerFaceColor','m');

% 绘制PSO路径
plot(path_coords_pso(:,1)/Lgrid, path_coords_pso(:,2)/Lgrid,'b-','LineWidth',1.5,'DisplayName','PSO轨迹');

legend('起点','终点','PSO轨迹','Location','best','FontName','华文中宋','FontSize',16);
title('PSO算法的轨迹图','FontName','宋体','FontSize',14);
grid off;
% save("path_coords_pso.mat", "path_coords_pso");
% 绘制适应度进化曲线
figure('Color','w');
set(gca,'FontName','Times New Roman','FontSize',12);
plot(1:max_iter, fitness_history, 'r-', 'LineWidth', 1.5);
xlabel('迭代次数','FontName','宋体','FontSize',14);
ylabel('最优适应度值','FontName','宋体','FontSize',14);
title('PSO算法适应度进化曲线','FontName','宋体','FontSize',16);
grid on;


% ================= 适应度函数 ==================
function fitness = calculate_fitness(position, gridMap, cellSize, gridSize)
    x = position(1,:);
    y = position(2,:);
    n = length(x);
    
    % 1. 路径长度成本
    length_cost = 0;
    for i = 1:n-1
        length_cost = length_cost + sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);
    end
    
    % 2. 障碍物成本
    obstacle_cost = 0;
    % 对路径进行插值，增加采样点以更好地检测碰撞
    interp_points = 20;  % 每段路径的插值点数
    for i = 1:n-1
        x_interp = linspace(x(i), x(i+1), interp_points);
        y_interp = linspace(y(i), y(i+1), interp_points);
        
        for j = 1:interp_points
            % 转换为栅格坐标
            grid_x = round(x_interp(j)/cellSize + 0.5);
            grid_y = round(y_interp(j)/cellSize + 0.5);
            
            % 检查是否在地图范围内
            if grid_x >= 1 && grid_x <= gridSize && grid_y >= 1 && grid_y <= gridSize
                % 检查是否为障碍物
                if gridMap(grid_y, grid_x) == 1
                    obstacle_cost = obstacle_cost + 1000;  % 碰撞惩罚
                else
                    % 距离障碍物越近惩罚越大
                    [dx, dy] = meshgrid(-2:2, -2:2);  % 检查周围5x5区域
                    for k = 1:numel(dx)
                        nx = grid_x + dx(k);
                        ny = grid_y + dy(k);
                        if nx >= 1 && nx <= gridSize && ny >= 1 && ny <= gridSize
                            if gridMap(ny, nx) == 1
                                dist = sqrt(dx(k)^2 + dy(k)^2);
                                obstacle_cost = obstacle_cost + 10 / (dist + 0.1);
                            end
                        end
                    end
                end
            else
                obstacle_cost = obstacle_cost + 1000;  % 出界惩罚
            end
        end
    end
    
    % 总适应度
    fitness = length_cost + obstacle_cost;
end


