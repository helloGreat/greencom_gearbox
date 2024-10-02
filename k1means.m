% 参数设置
num_users = 12;  % 用户数量
num_groups = 4;  % 组数
theta_range = 120; % 基站范围（度数）

% 生成用户坐标 (基站假设在(0,0))
r = 100; % 假设用户距离基站的最大距离为100
angles = rand(num_users, 1) * deg2rad(theta_range); % 随机生成角度 (弧度)
x = r * cos(angles);  % x 坐标
y = r * sin(angles);  % y 坐标

% 计算矢量方向 (角度)
directions = atan2(y, x);  % atan2 返回的是弧度制角度

% 使用 kmeans 按照方向进行聚类
[idx, ~] = kmeans(directions, num_groups);

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
