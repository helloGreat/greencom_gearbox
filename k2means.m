% 参数设置
num_users = 50;  % 用户数量
max_groups = 4;  % 允许的最大组数
theta_range = 120; % 基站范围（度数）

% 生成用户坐标 (假设基站在(0,0))
r = 100 * rand(num_users, 1); % 随机生成用户距离基站的距离
angles = rand(num_users, 1) * deg2rad(theta_range); % 随机生成角度 (弧度)
x = r .* cos(angles);  % x 坐标
y = r .* sin(angles);  % y 坐标

% 计算矢量方向 (角度)
directions = atan2(y, x);  % 计算方向角，单位为弧度

% 构建特征向量：将方向角（弧度）和距离作为特征
features = [directions, r];

% 使用 kmeans 聚类，同时考虑方向和距离
optimal_num_groups = min(num_users, max_groups);  % 组数不超过用户数或最大组数
[idx, C] = kmeans(features, optimal_num_groups, 'Replicates', 100);

% 可视化用户分布和分组结果
figure;
gscatter(x, y, idx); % 根据聚类结果对用户进行不同颜色的标记
hold on;
plot(0, 0, 'kx', 'MarkerSize', 15, 'LineWidth', 2); % 基站位置
title('用户分布及分组结果');
xlabel('X 坐标');
ylabel('Y 坐标');
axis equal;
grid on;
hold off;
