%% 多平面RANSAC拟合算法
% 使用RANSAC算法从点云中拟合多个平面
% 参考改进的多平面检测逻辑

% % 角焊缝：
% % 点云加载完成，共 52065 个点
% % 
% % RANSAC参数设置：
% % 最大迭代次数: 1000
% % 距离阈值: 0.3000
% % 最小内点数: 1000
% % 最大平面数: 2
% % 内点比例阈值: 0.40 (>= 此值时停止检测)
% % 
% % 开始多平面RANSAC拟合...
% % 
% % ========== 检测第 1 个平面 ==========
% % 当前点云数量: 52065
% % 迭代 200/1000, 当前最佳内点数: 19008
% % 迭代 400/1000, 当前最佳内点数: 19008
% % 迭代 600/1000, 当前最佳内点数: 19022
% % 迭代 800/1000, 当前最佳内点数: 19022
% % 迭代 1000/1000, 当前最佳内点数: 19161
% % RANSAC拟合完成，耗时: 0.15 秒
% % 最佳内点数: 19161 / 52065 (36.80%)
% % 使用所有内点进行最小二乘优化...
% % 平面 1 方程: -0.0110x + 0.8220y + -0.5694z + 251.0011 = 0
% % 平面 1 法向量: [-0.0110, 0.8220, -0.5694]
% % 移除当前平面内点后，剩余点数: 32904
% % 内点比例 36.80% < 阈值 40.00%，可能存在多个平面，继续检测...
% % 
% % ========== 检测第 2 个平面 ==========
% % 当前点云数量: 32904
% % 迭代 200/1000, 当前最佳内点数: 16082
% % 迭代 400/1000, 当前最佳内点数: 16082
% % 迭代 600/1000, 当前最佳内点数: 16082
% % 迭代 800/1000, 当前最佳内点数: 16091
% % 迭代 1000/1000, 当前最佳内点数: 16091
% % RANSAC拟合完成，耗时: 0.12 秒
% % 最佳内点数: 16091 / 32904 (48.90%)
% % 使用所有内点进行最小二乘优化...
% % 平面 2 方程: -0.0162x + -0.6040y + -0.7968z + 339.0820 = 0
% % 平面 2 法向量: [-0.0162, -0.6040, -0.7968]
% % 移除当前平面内点后，剩余点数: 16813
% % 内点比例 48.90% >= 阈值 40.00%，判定为单平面场景，停止检测
% % 
% % ========== 检测完成 ==========
% % 共检测到 2 个平面
% % 总耗时: 0.27 秒
% % 
% % 平面 1 详细信息：
% %   方程: -0.0110x + 0.8220y + -0.5694z + 251.0011 = 0
% %   内点数: 19161
% %   质心: [55.7465, 1.2416, 441.5525]
% % 
% % 平面 2 详细信息：
% %   方程: -0.0162x + -0.6040y + -0.7968z + 339.0820 = 0
% %   内点数: 16091
% %   质心: [61.6480, -14.9409, 435.6100]
% % 
% % 生成可视化结果...
% % 
% % 是否保存拟合结果？
% % 输入 y/Y 保存，输入 n/N 不保存: n
% % 已取消保存。
% % 
% % 程序运行完成！
% % 

% % 平面焊缝
% % 点云加载完成，共 45979 个点
% % 
% % RANSAC参数设置：
% % 最大迭代次数: 1000
% % 距离阈值: 0.3000
% % 最小内点数: 1000
% % 最大平面数: 2
% % 内点比例阈值: 0.40 (>= 此值时停止检测)
% % 
% % 开始多平面RANSAC拟合...
% % 
% % ========== 检测第 1 个平面 ==========
% % 当前点云数量: 45979
% % 迭代 200/1000, 当前最佳内点数: 29953
% % 迭代 400/1000, 当前最佳内点数: 30145
% % 迭代 600/1000, 当前最佳内点数: 30145
% % 迭代 800/1000, 当前最佳内点数: 30145
% % 迭代 1000/1000, 当前最佳内点数: 30145
% % RANSAC拟合完成，耗时: 0.20 秒
% % 最佳内点数: 30145 / 45979 (65.56%)
% % 使用所有内点进行最小二乘优化...
% % 平面 1 方程: 0.0087x + -0.0014y + 1.0000z + -18.3157 = 0
% % 平面 1 法向量: [0.0087, -0.0014, 1.0000]
% % 移除当前平面内点后，剩余点数: 15834
% % 内点比例 65.56% >= 阈值 40.00%，判定为单平面场景，停止检测
% % 
% % ========== 检测完成 ==========
% % 共检测到 1 个平面
% % 总耗时: 0.21 秒
% % 
% % 平面 1 详细信息：
% %   方程: 0.0087x + -0.0014y + 1.0000z + -18.3157 = 0
% %   内点数: 30145
% %   质心: [95.9964, 29.7292, 17.5246]
% % 
% % 生成可视化结果...
% % 
% % 是否保存拟合结果？
% % 输入 y/Y 保存，输入 n/N 不保存: n
% % 已取消保存。
% % 
% % 程序运行完成！


clc
clear
close all;

%% 读取点云数据
[fileName, pathName] = uigetfile({'*.pcd';'*.txt';'*.ply'}, '选择点云文件');

if isempty(fileName) || length(fileName) == 1
    fprintf("未选择点云文件！\n");
    return;
end

% 读取点云
pc = pcread([pathName, fileName]);
Data = pc.Location;  % 获取点云的位置信息 [x, y, z]
fprintf('点云加载完成，共 %d 个点\n', size(Data, 1));

%% RANSAC参数设置
maxIterations = 1000;       % 最大迭代次数
distanceThreshold = 0.4;    % 内点距离阈值（根据点云尺度调整）
minInliers = 1000;          % 最小内点数量
maxPlanes = 2;              % 最多检测的平面数量（作为上限保护）
inlierRatioThreshold = 0.40; % 内点比例阈值：如果内点比例>=此值，认为只有一个平面

fprintf('\nRANSAC参数设置：\n');
fprintf('最大迭代次数: %d\n', maxIterations);
fprintf('距离阈值: %.4f\n', distanceThreshold);
fprintf('最小内点数: %d\n', minInliers);
fprintf('最大平面数: %d\n', maxPlanes);
fprintf('内点比例阈值: %.2f (>= 此值时停止检测)\n', inlierRatioThreshold);

%% 多平面检测主循环
fprintf('\n开始多平面RANSAC拟合...\n');

allPlanes = {};  % 存储所有检测到的平面信息
remainingIndices = (1:size(Data, 1))';  % 剩余点的索引（列向量）
totalStartTime = tic;

for planeIdx = 1:maxPlanes
    fprintf('\n========== 检测第 %d 个平面 ==========\n', planeIdx);
    
    % 检查剩余点数是否足够
    if length(remainingIndices) < minInliers
        fprintf('剩余点数 %d 不足最小内点数 %d，停止检测\n', length(remainingIndices), minInliers);
        break;
    end
    
    % 当前待处理的点云数据
    currentData = Data(remainingIndices, :);
    numPoints = size(currentData, 1);
    fprintf('当前点云数量: %d\n', numPoints);
    
    %% 执行RANSAC算法
    tic;
    bestInliers = [];
    bestModel = [];
    bestInlierCount = 0;
    
    for iter = 1:maxIterations
        % 随机选择3个点
        randomIndices = randperm(numPoints, 3);
        p1 = currentData(randomIndices(1), :);
        p2 = currentData(randomIndices(2), :);
        p3 = currentData(randomIndices(3), :);
        
        % 计算平面方程：Ax + By + Cz + D = 0
        v1 = p2 - p1;
        v2 = p3 - p1;
        normal = cross(v1, v2);  % 法向量
        
        % 检查法向量是否有效
        if norm(normal) < 1e-6
            continue;
        end
        
        normal = normal / norm(normal);  % 归一化
        A = normal(1);
        B = normal(2);
        C = normal(3);
        D = -(A*p1(1) + B*p1(2) + C*p1(3));
        
        % 计算所有点到平面的距离
        distances = abs(currentData(:,1)*A + currentData(:,2)*B + currentData(:,3)*C + D) / sqrt(A^2 + B^2 + C^2);
        
        % 统计内点
        inliers = find(distances < distanceThreshold);
        inlierCount = length(inliers);
        
        % 更新最佳模型
        if inlierCount > bestInlierCount
            bestInlierCount = inlierCount;
            bestInliers = inliers;
            bestModel = [A, B, C, D];
        end
        
        % 显示进度
        if mod(iter, 200) == 0
            fprintf('迭代 %d/%d, 当前最佳内点数: %d\n', iter, maxIterations, bestInlierCount);
        end
    end
    
    elapsedTime = toc;
    fprintf('RANSAC拟合完成，耗时: %.2f 秒\n', elapsedTime);
    
    % 计算内点比例
    inlierRatio = bestInlierCount / numPoints;
    fprintf('最佳内点数: %d / %d (%.2f%%)\n', bestInlierCount, numPoints, 100*inlierRatio);
    
    %% 检查是否找到有效平面
    if bestInlierCount < minInliers
        fprintf('内点数量不足，未找到更多有效平面\n');
        break;
    end
    
    %% 使用所有内点重新拟合平面（最小二乘优化）
    fprintf('使用所有内点进行最小二乘优化...\n');
    inlierPoints = currentData(bestInliers, :);
    
    % 使用SVD进行平面拟合
    centroid = mean(inlierPoints, 1);
    centeredPoints = inlierPoints - centroid;
    [~, ~, V] = svd(centeredPoints, 0);
    normal = V(:, 3)';  % 最小特征值对应的特征向量
    
    % 更新平面方程
    A = normal(1);
    B = normal(2);
    C = normal(3);
    D = -(A*centroid(1) + B*centroid(2) + C*centroid(3));
    
    fprintf('平面 %d 方程: %.4fx + %.4fy + %.4fz + %.4f = 0\n', planeIdx, A, B, C, D);
    fprintf('平面 %d 法向量: [%.4f, %.4f, %.4f]\n', planeIdx, A, B, C);
    
    % 保存平面信息（转换为原始点云索引）
    globalInliers = remainingIndices(bestInliers);
    allPlanes{planeIdx} = struct(...
        'model', [A, B, C, D], ...
        'inliers', globalInliers, ...
        'inlierCount', bestInlierCount, ...
        'centroid', centroid, ...
        'normal', [A, B, C]);
    
    % 从剩余点中移除当前平面的内点
    remainingIndices(bestInliers) = [];
    fprintf('移除当前平面内点后，剩余点数: %d\n', length(remainingIndices));
    
    %% 判断是否需要继续检测下一个平面
    if inlierRatio >= inlierRatioThreshold
        fprintf('内点比例 %.2f%% >= 阈值 %.2f%%，判定为单平面场景，停止检测\n', ...
                100*inlierRatio, 100*inlierRatioThreshold);
        break;
    else
        fprintf('内点比例 %.2f%% < 阈值 %.2f%%，可能存在多个平面，继续检测...\n', ...
                100*inlierRatio, 100*inlierRatioThreshold);
    end
end

totalElapsedTime = toc(totalStartTime);
numPlanesDetected = length(allPlanes);

fprintf('\n========== 检测完成 ==========\n');
fprintf('共检测到 %d 个平面\n', numPlanesDetected);
fprintf('总耗时: %.2f 秒\n', totalElapsedTime);

%% 检查是否检测到平面
if numPlanesDetected == 0
    fprintf('警告：未找到任何有效平面！\n');
    return;
end

% 显示所有平面的详细信息
for i = 1:numPlanesDetected
    planeInfo = allPlanes{i};
    fprintf('\n平面 %d 详细信息：\n', i);
    fprintf('  方程: %.4fx + %.4fy + %.4fz + %.4f = 0\n', planeInfo.model);
    fprintf('  内点数: %d\n', planeInfo.inlierCount);
    fprintf('  质心: [%.4f, %.4f, %.4f]\n', planeInfo.centroid);
end

%% 可视化结果
fprintf('\n生成可视化结果...\n');

% 创建图形
figure('Name', '多平面RANSAC拟合结果', 'Position', [100, 100, 1400, 600]);

% 子图1：原始点云
subplot(1, 2, 1);
pcshow(pc);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
axis equal;

% 子图2：多平面拟合结果
subplot(1, 2, 2);
hold on;
grid on;
axis equal;
rotate3d on;

% 定义颜色映射
colorMap = [1 0 0;      % 红色 - 平面1
            0 1 0;      % 绿色 - 平面2
            0 0 1;      % 蓝色 - 平面3
            1 1 0;      % 黄色 - 平面4
            1 0 1;      % 品红 - 平面5
            0 1 1];     % 青色 - 平面6

legendEntries = {};
plotHandles = [];

% 绘制每个检测到的平面
for i = 1:numPlanesDetected
    planeInfo = allPlanes{i};
    inlierData = Data(planeInfo.inliers, :);
    color = colorMap(mod(i-1, size(colorMap, 1)) + 1, :);
    
    % 绘制内点
    h = plot3(inlierData(:,1), inlierData(:,2), inlierData(:,3), ...
              '.', 'MarkerSize', 4, 'Color', color);
    plotHandles(end+1) = h;
    legendEntries{end+1} = sprintf('平面 %d (%d点)', i, planeInfo.inlierCount);
    
    % 绘制拟合平面
    A = planeInfo.model(1);
    B = planeInfo.model(2);
    C = planeInfo.model(3);
    D = planeInfo.model(4);
    
    % 计算该平面内点的范围
    xRange = [min(inlierData(:,1)), max(inlierData(:,1))];
    yRange = [min(inlierData(:,2)), max(inlierData(:,2))];
    
    % 扩展范围以便更好地可视化
    xPadding = (xRange(2) - xRange(1)) * 0.1;
    yPadding = (yRange(2) - yRange(1)) * 0.1;
    xRange = xRange + [-xPadding, xPadding];
    yRange = yRange + [-yPadding, yPadding];
    
    [xGrid, yGrid] = meshgrid(linspace(xRange(1), xRange(2), 30), ...
                               linspace(yRange(1), yRange(2), 30));
    
    if abs(C) > 1e-6
        zGrid = -(A*xGrid + B*yGrid + D) / C;
        surf(xGrid, yGrid, zGrid, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
             'FaceColor', color);
    end
    
    % 绘制法向量（从质心出发）
    quiver3(planeInfo.centroid(1), planeInfo.centroid(2), planeInfo.centroid(3), ...
            planeInfo.normal(1), planeInfo.normal(2), planeInfo.normal(3), ...
            3, 'k', 'LineWidth', 2);
end

% 绘制未分类的点（外点）
if ~isempty(remainingIndices)
    outlierData = Data(remainingIndices, :);
    h = plot3(outlierData(:,1), outlierData(:,2), outlierData(:,3), ...
              'k.', 'MarkerSize', 2);
    plotHandles(end+1) = h;
    legendEntries{end+1} = sprintf('外点 (%d点)', length(remainingIndices));
end

title(sprintf('多平面拟合结果 (共检测到%d个平面)', numPlanesDetected));
xlabel('X'); ylabel('Y'); zlabel('Z');
legend(plotHandles, legendEntries, 'Location', 'best');

%% 保存结果
fprintf('\n是否保存拟合结果？\n');
userChoice = input('输入 y/Y 保存，输入 n/N 不保存: ', 's');

if strcmpi(userChoice, 'y')
    % 为每个平面保存点云文件
    for i = 1:numPlanesDetected
        planeInfo = allPlanes{i};
        inlierData = Data(planeInfo.inliers, :);
        inlierPC = pointCloud(inlierData);
        
        % 生成文件名
        [~, baseName, ~] = fileparts(fileName);
        outputFileName = fullfile(pathName, sprintf('%s_plane_%d.ply', baseName, i));
        pcwrite(inlierPC, outputFileName);
        fprintf('平面 %d 点云已保存: %s\n', i, outputFileName);
    end
    
    % 保存所有平面参数到一个文本文件
    [~, baseName, ~] = fileparts(fileName);
    resultFile = fullfile(pathName, sprintf('%s_多平面拟合结果.txt', baseName));
    fid = fopen(resultFile, 'w', 'n', 'UTF-8');
    fprintf(fid, '多平面RANSAC拟合结果\n');
    fprintf(fid, '====================\n\n');
    fprintf(fid, '检测到的平面数量: %d\n', numPlanesDetected);
    fprintf(fid, '总点数: %d\n', size(Data, 1));
    fprintf(fid, '外点数量: %d\n', length(remainingIndices));
    fprintf(fid, '总耗时: %.2f 秒\n\n', totalElapsedTime);
    
    fprintf(fid, 'RANSAC参数设置:\n');
    fprintf(fid, '  最大迭代次数: %d\n', maxIterations);
    fprintf(fid, '  距离阈值: %.6f\n', distanceThreshold);
    fprintf(fid, '  最小内点数: %d\n', minInliers);
    fprintf(fid, '  内点比例阈值: %.2f\n\n', inlierRatioThreshold);
    
    fprintf(fid, '========================================\n\n');
    
    % 为每个平面保存详细信息
    for i = 1:numPlanesDetected
        planeInfo = allPlanes{i};
        fprintf(fid, '平面 %d:\n', i);
        fprintf(fid, '--------\n');
        fprintf(fid, '  方程: %.6fx + %.6fy + %.6fz + %.6f = 0\n', planeInfo.model);
        fprintf(fid, '  法向量: [%.6f, %.6f, %.6f]\n', planeInfo.normal);
        fprintf(fid, '  质心: [%.6f, %.6f, %.6f]\n', planeInfo.centroid);
        fprintf(fid, '  内点数量: %d\n', planeInfo.inlierCount);
        fprintf(fid, '  内点比例: %.2f%%\n', 100*planeInfo.inlierCount/size(Data, 1));
        fprintf(fid, '\n');
    end
    
    fclose(fid);
    fprintf('拟合参数已保存: %s\n', resultFile);
else
    fprintf('已取消保存。\n');
end

fprintf('\n程序运行完成！\n');

