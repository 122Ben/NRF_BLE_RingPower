clear;
clc;
% 你的输入数据
x = [0 5 10 20 30 40 50 60 70 80 90 100];
y = [3 3.45 3.68 3.74 3.77 3.79 3.82 3.87 3.92 3.98 4.06 4.2];

% 选择一个拟合多项式的程度，例如二次多项式
degree = 7;

% 对数据进行多项式拟合，返回拟合参数
p = polyfit(x, y, degree);

% 在拟合的多项式上计算 y 的值
xFit = linspace(min(x), max(x), 1000); % 生成密集的点以绘制光滑曲线
yFit = polyval(p, xFit);

% 新的 x 值
x_new = [0 12.5 25 37.5 50 62.5 75.0 87.5 100];

% 使用拟合的多项式系数计算新 x 值对应的 y 值
y_new = polyval(p, x_new);

% 绘制原始数据点
figure; % 新建一个图形窗口
plot(x, y, 'o', 'MarkerSize', 8, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b'); % 用蓝色圆圈标记原始数据点
hold on; % 保持图形，以便在相同图形上绘制拟合曲线

% 绘制拟合曲线
plot(xFit, yFit, 'r-', 'LineWidth', 2); % 用红色实线绘制拟合曲线

% 添加图例和坐标轴标签
legend('原始数据', '拟合曲线');
xlabel('x');
ylabel('y');
title('多项式拟合');
grid on;  % 添加网格线以方便观察

% 保持图形打开状态
hold off;
