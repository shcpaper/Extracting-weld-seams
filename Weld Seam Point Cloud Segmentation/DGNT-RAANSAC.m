%% 改进的RANSAC平面拟合算法（基于法向量约束，支持多平面检测）
% 使用RANSAC算法从点云中拟合平面
% 改进点1：使用距离阈值和法向量角度阈值双重约束判定内点
% 改进点2：支持迭代检测多个平面

% % 角焊缝
% % 点云加载完成，共 52065 个点
% % 正在计算点云法向量...
% % 法向量计算完成，耗时: 0.19 秒
% % 
% % RANSAC参数设置：
% % 最大迭代次数: 1000
% % 距离阈值: 0.3000
% % 法向量角度阈值: 10.00 度
% % 最小内点数: 1000
% % 最大平面数: 5
% % 候选点采样数量: 500
% % 内点比例阈值: 0.40 (>= 此值时停止检测)
% % 
% % 开始多平面RANSAC拟合...
% % 
% % ========== 检测第 1 个平面 ==========
% % 当前点云数量: 52065
% % 迭代 200/1000, 当前最佳内点数: 18045
% % 迭代 400/1000, 当前最佳内点数: 18145
% % 迭代 600/1000, 当前最佳内点数: 18145
% % 迭代 800/1000, 当前最佳内点数: 18145
% % 迭代 1000/1000, 当前最佳内点数: 18145
% % RANSAC拟合完成，耗时: 0.35 秒
% % 最佳内点数: 18145 / 52065 (34.85%)
% % 使用所有内点进行最小二乘优化...
% % 平面 1 方程: -0.0107x + 0.8231y + -0.5677z + 250.2684 = 0
% % 平面 1 法向量: [-0.0107, 0.8231, -0.5677]
% % 移除当前平面内点后，剩余点数: 33920
% % 内点比例 34.85% < 阈值 40.00%，可能存在多个平面，继续检测...
% % 
% % ========== 检测第 2 个平面 ==========
% % 当前点云数量: 33920
% % 迭代 200/1000, 当前最佳内点数: 14646
% % 迭代 400/1000, 当前最佳内点数: 14646
% % 迭代 600/1000, 当前最佳内点数: 14647
% % 迭代 800/1000, 当前最佳内点数: 14647
% % 迭代 1000/1000, 当前最佳内点数: 14647
% % RANSAC拟合完成，耗时: 0.23 秒
% % 最佳内点数: 14647 / 33920 (43.18%)
% % 使用所有内点进行最小二乘优化...
% % 平面 2 方程: -0.0160x + -0.6091y + -0.7929z + 337.2831 = 0
% % 平面 2 法向量: [-0.0160, -0.6091, -0.7929]
% % 移除当前平面内点后，剩余点数: 19273
% % 内点比例 43.18% >= 阈值 40.00%，判定为单平面场景，停止检测
% % 
% % ========== 检测完成 ==========
% % 共检测到 2 个平面
% % 总耗时: 0.59 秒
% % 
% % 平面 1 详细信息：
% %   方程: -0.0107x + 0.8231y + -0.5677z + 250.2684 = 0
% %   内点数: 18145
% %   质心: [56.6262, 1.5683, 442.0182]
% % 
% % 平面 2 详细信息：
% %   方程: -0.0160x + -0.6091y + -0.7929z + 337.2831 = 0
% %   内点数: 14647
% %   质心: [61.8901, -15.3075, 435.8682]
% % 
% % 生成可视化结果...
% % 
% % 是否保存拟合结果？
% % 输入 y/Y 保存，输入 n/N 不保存: n
% % 已取消保存。
% % 
% % 程序运行完成！

% % 平面焊缝
% % 点云加载完成，共 45979 个点
% % 正在计算点云法向量...
% % 法向量计算完成，耗时: 0.15 秒
% % 
% % RANSAC参数设置：
% % 最大迭代次数: 1000
% % 距离阈值: 0.3000
% % 法向量角度阈值: 10.00 度
% % 最小内点数: 1000
% % 最大平面数: 5
% % 候选点采样数量: 500
% % 内点比例阈值: 0.40 (>= 此值时停止检测)
% % 
% % 开始多平面RANSAC拟合...
% % 
% % ========== 检测第 1 个平面 ==========
% % 当前点云数量: 45979
% % 迭代 200/1000, 当前最佳内点数: 26976
% % 迭代 400/1000, 当前最佳内点数: 26976
% % 迭代 600/1000, 当前最佳内点数: 27030
% % 迭代 800/1000, 当前最佳内点数: 27071
% % 迭代 1000/1000, 当前最佳内点数: 27071
% % RANSAC拟合完成，耗时: 0.44 秒
% % 最佳内点数: 27071 / 45979 (58.88%)
% % 使用所有内点进行最小二乘优化...
% % 平面 1 方程: 0.0082x + -0.0014y + 1.0000z + -18.2420 = 0
% % 平面 1 法向量: [0.0082, -0.0014, 1.0000]
% % 移除当前平面内点后，剩余点数: 18908
% % 内点比例 58.88% >= 阈值 40.00%，判定为单平面场景，停止检测
% % 
% % ========== 检测完成 ==========
% % 共检测到 1 个平面
% % 总耗时: 0.44 秒
% % 
% % 平面 1 详细信息：
% %   方程: 0.0082x + -0.0014y + 1.0000z + -18.2420 = 0
% %   内点数: 27071
% %   质心: [95.9020, 30.9840, 17.5018]
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

%% 计算点云法向量
fprintf('正在计算点云法向量...\n');
tic;
% 使用k近邻计算法向量
k = 20;  % 近邻点数量
normals = pcnormals(pc, k);
normalTime = toc;
fprintf('法向量计算完成，耗时: %.2f 秒\n', normalTime);

%% RANSAC参数设置
% 平面点云
% maxIterations = 1000;      % 最大迭代次数
% distanceThreshold = 0.25;  % 内点距离阈值（根据点云尺度调整）
% normalAngleThreshold = 10; % 法向量角度阈值（度）
% minInliers = 1000;          % 最小内点数量

maxIterations = 1000;       % 最大迭代次数
distanceThreshold = 0.3;    % 内点距离阈值（根据点云尺度调整）
normalAngleThreshold = 10;  % 法向量角度阈值（度）
minInliers = 1000;          % 最小内点数量
maxPlanes = 2;              % 最多检测的平面数量（作为上限保护）
sampleSize = 500;           % 采样候选点数量（用于加速法向量筛选）
inlierRatioThreshold = 0.40; % 内点比例阈值：如果内点比例>=此值，认为只有一个平面

fprintf('\nRANSAC参数设置：\n');
fprintf('最大迭代次数: %d\n', maxIterations);
fprintf('距离阈值: %.4f\n', distanceThreshold);
fprintf('法向量角度阈值: %.2f 度\n', normalAngleThreshold);
fprintf('最小内点数: %d\n', minInliers);
fprintf('最大平面数: %d\n', maxPlanes);
fprintf('候选点采样数量: %d\n', sampleSize);
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
    currentNormals = normals(remainingIndices, :);
    numPoints = size(currentData, 1);
    fprintf('当前点云数量: %d\n', numPoints);
    
    %% 执行RANSAC算法
    tic;
    bestInliers = [];
    bestModel = [];
    bestInlierCount = 0;
    
    for iter = 1:maxIterations
        % 随机选择第一个点
        firstIdx = randi(numPoints);
        p1 = Data(firstIdx, :);
        n1 = normals(firstIdx, :);  % 第一个点的法向量
        
        % 【优化】只从随机采样的点中寻找候选点，而不是遍历所有点
        % 确定实际采样数量（不超过总点数）
        actualSampleSize = min(sampleSize, numPoints);
        
        % 随机采样一部分点作为候选池
        sampleIndices = randperm(numPoints, actualSampleSize);
        
        % 计算采样点的法向量与第一个点法向量的角度差
        sampledNormals = normals(sampleIndices, :);
        dotProducts = abs(sampledNormals * n1');  % 点积的绝对值
        angles = acosd(dotProducts);  % 转换为角度
        
        % 筛选满足法向量角度阈值的候选点
        validMask = angles < normalAngleThreshold;
        candidateIndices = sampleIndices(validMask);
        
        % 移除第一个点自己（如果在候选点中）
        candidateIndices(candidateIndices == firstIdx) = [];
        
        % 检查是否有足够的候选点
        if length(candidateIndices) < 2
            continue;  % 候选点不足，跳过此次迭代
        end
        
        % 从候选点中随机选择另外两个点
        selectedIndices = randperm(length(candidateIndices), 2);
        secondIdx = candidateIndices(selectedIndices(1));
        thirdIdx = candidateIndices(selectedIndices(2));
        
        p2 = Data(secondIdx, :);
        p3 = Data(thirdIdx, :);
        
            
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
        
        % 计算法向量角度差异
        dotProducts = abs(currentNormals(:,1)*A + currentNormals(:,2)*B + currentNormals(:,3)*C);
        angles = acosd(dotProducts);  % 角度范围 [0, 90]
        
        % 统计内点（同时满足距离和法向量约束）
        inliers = find(distances < distanceThreshold & angles < normalAngleThreshold);
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
    fprintf(fid, '  法向量角度阈值: %.2f 度\n', normalAngleThreshold);
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

