%% 区域生长算法 - 用于区分平面和焊缝
% 基于法向量和曲率的区域生长分割
clc
clear
close all;

%% 读取点云数据
fprintf('========== 区域生长点云分割 ==========\n\n');
[fileName, pathName] = uigetfile({'*.pcd';'*.txt';'*.ply'}, '选择点云文件');

if isempty(fileName) || length(fileName) == 1
    fprintf("未选择点云文件！\n");
    return;
end

% 读取点云
pc = pcread([pathName, fileName]);
Data = pc.Location;  % 获取点云的位置信息 [x, y, z]
numPoints = size(Data, 1);
fprintf('点云加载完成，共 %d 个点\n', numPoints);

%% 区域生长参数设置
fprintf('\n---------- 参数设置 ----------\n');

% 法向量计算参数
kNeighbors = 20;  % 计算法向量和曲率时使用的邻近点数
fprintf('法向量邻近点数: %d\n', kNeighbors);

% 区域生长参数
normalAngleThreshold = 1;  % 法向量角度阈值（度）- 平面区域使用较小值
curvatureThreshold = 0.01;  % 曲率阈值 - 平面区域曲率应该较小
searchRadius = 2.0;  % 邻域搜索半径（根据点云密度调整）
minRegionSize = 1000;  % 最小区域点数

% 区域分类方式参数
% classifyMethod: 1=自动分类(基于曲率和法向量), 2=手动指定前N个为平面
classifyMethod = 2;  % 分类方式
nPlaneRegions = 2;   % 当classifyMethod=2时，指定前N个点数最多的区域为平面

fprintf('法向量角度阈值: %.2f 度\n', normalAngleThreshold);
fprintf('曲率阈值: %.4f\n', curvatureThreshold);
fprintf('搜索半径: %.4f\n', searchRadius);
fprintf('最小区域点数: %d\n', minRegionSize);
fprintf('分类方式: %d (1=自动, 2=手动前N个)\n', classifyMethod);
if classifyMethod == 2
    fprintf('选择前 %d 个点数最多的区域为平面\n', nPlaneRegions);
end

%% 计算法向量和曲率
fprintf('\n正在计算点云法向量和曲率...\n');
tic;

% 使用pcnormals计算法向量
normals = pcnormals(pc, kNeighbors);

% 计算曲率（使用协方差矩阵的特征值）
curvatures = zeros(numPoints, 1);
kdtree = KDTreeSearcher(Data);

fprintf('正在计算曲率...\n');
for i = 1:numPoints
    if mod(i, 1000) == 0
        fprintf('  进度: %d/%d (%.1f%%)\n', i, numPoints, 100*i/numPoints);
    end
    
    % 找到邻近点
    [indices, ~] = knnsearch(kdtree, Data(i, :), 'K', kNeighbors);
    neighbors = Data(indices, :);
    
    % 计算协方差矩阵
    centered = neighbors - mean(neighbors, 1);
    cov_matrix = (centered' * centered) / (kNeighbors - 1);
    
    % 计算特征值
    eigenvalues = eig(cov_matrix);
    eigenvalues = sort(eigenvalues, 'ascend');
    
    % 曲率定义为最小特征值与所有特征值之和的比值
    if sum(eigenvalues) > 1e-10
        curvatures(i) = eigenvalues(1) / sum(eigenvalues);
    else
        curvatures(i) = 0;
    end
end

toc;
fprintf('法向量和曲率计算完成！\n');

%% 区域生长算法
fprintf('\n---------- 开始区域生长分割 ----------\n');
tic;

% 初始化
regions = zeros(numPoints, 1);  % 每个点所属的区域ID，0表示未分配
regionCount = 0;  % 区域计数
availablePoints = true(numPoints, 1);  % 可用点标记

% 按曲率排序，优先处理曲率小的点（平面区域的种子点）
[sortedCurvatures, sortedIndices] = sort(curvatures);

% 区域生长主循环
while any(availablePoints)
    % 选择曲率最小的未分配点作为种子点
    seedIdx = [];
    for i = 1:numPoints
        idx = sortedIndices(i);
        if availablePoints(idx)
            seedIdx = idx;
            break;
        end
    end
    
    if isempty(seedIdx)
        break;
    end
    
    % 创建新区域
    regionCount = regionCount + 1;
    currentRegion = seedIdx;
    regions(seedIdx) = regionCount;
    availablePoints(seedIdx) = false;
    
    % 种子点列表（待处理的点）
    seeds = [seedIdx];
    seedPointer = 1;
    
    fprintf('  区域 %d: ', regionCount);
    
    % 区域生长
    while seedPointer <= length(seeds)
        currentSeed = seeds(seedPointer);
        seedPointer = seedPointer + 1;
        
        % 找到当前种子点的邻近点
        [indices, distances] = rangesearch(kdtree, Data(currentSeed, :), searchRadius);
        neighborIndices = indices{1};
        neighborDistances = distances{1};
        
        % 遍历每个邻近点
        for j = 1:length(neighborIndices)
            neighborIdx = neighborIndices(j);
            
            % 跳过已分配的点和当前种子点本身
            if ~availablePoints(neighborIdx) || neighborIdx == currentSeed
                continue;
            end
            
            % 检查法向量角度
            normalAngle = acosd(abs(dot(normals(currentSeed, :), normals(neighborIdx, :))));
            
            % 检查是否满足区域生长条件
            if normalAngle < normalAngleThreshold && curvatures(neighborIdx) < curvatureThreshold
                % 将邻近点加入当前区域
                regions(neighborIdx) = regionCount;
                availablePoints(neighborIdx) = false;
                currentRegion = [currentRegion; neighborIdx];
                
                % 如果邻近点的曲率足够小，将其作为新的种子点
                if curvatures(neighborIdx) < curvatureThreshold * 0.8
                    seeds = [seeds; neighborIdx];
                end
            end
        end
    end
    
    fprintf('包含 %d 个点\n', length(currentRegion));
end

toc;
fprintf('\n区域生长完成！\n');
fprintf('共生成 %d 个区域\n', regionCount);

%% 过滤小区域
fprintf('\n正在过滤小于 %d 个点的小区域...\n', minRegionSize);
validRegions = [];
for i = 1:regionCount
    regionSize = sum(regions == i);
    if regionSize >= minRegionSize
        validRegions = [validRegions; i];
    else
        % 将小区域标记为未分类（设为0）
        regions(regions == i) = 0;
    end
end

% 重新编号有效区域
newRegions = zeros(numPoints, 1);
for i = 1:length(validRegions)
    newRegions(regions == validRegions(i)) = i;
end
regions = newRegions;
regionCount = length(validRegions);

fprintf('过滤后剩余 %d 个有效区域\n', regionCount);

%% 区域排序（按点数）
fprintf('\n---------- 区域点数统计 ----------\n');
regionSizes = zeros(regionCount, 1);
for i = 1:regionCount
    regionSizes(i) = sum(regions == i);
    fprintf('  区域 %d: %d 个点\n', i, regionSizes(i));
end

% 按点数降序排序
[sortedSizes, sortedRegionIndices] = sort(regionSizes, 'descend');

%% 应用分类方式
fprintf('\n---------- 应用区域分类方式 ----------\n');

if classifyMethod == 2
    % 手动指定前N个为平面
    fprintf('使用手动分类方式: 前 %d 个点数最多的区域为平面\n', nPlaneRegions);
    fprintf('\n当前共有 %d 个区域\n', regionCount);
    fprintf('区域点数排序（从多到少）：\n');
    for i = 1:min(10, regionCount)  % 显示前10个
        regionID = sortedRegionIndices(i);
        fprintf('  第 %d 名: 区域 %d (%d 个点)\n', i, regionID, sortedSizes(i));
    end
    
    % 验证nPlaneRegions参数
    if nPlaneRegions < 1 || nPlaneRegions > regionCount
        fprintf('\n警告: nPlaneRegions=%d 超出范围，自动调整为: %d\n', ...
            nPlaneRegions, min(2, regionCount));
        nPlaneRegions = min(2, regionCount);
    end
    
    fprintf('\n已选择前 %d 个点数最多的区域作为平面\n', nPlaneRegions);
    
    % 标记区域类型
    regionTypes = strings(regionCount, 1);
    regionTypes(:) = "焊缝";  % 默认全部为焊缝
    
    % 将前N个点数最多的区域标记为平面
    planeRegionIDs = sortedRegionIndices(1:nPlaneRegions);
    for i = 1:length(planeRegionIDs)
        regionID = planeRegionIDs(i);
        regionTypes(regionID) = "平面";
        fprintf('  区域 %d -> 平面 (%d 个点)\n', regionID, regionSizes(regionID));
    end
else
    % 自动分类方式
    fprintf('使用自动分类方式（基于曲率和法向量）\n');
    regionTypes = strings(regionCount, 1);
end

%% 区域分类：平面 vs 焊缝
fprintf('\n---------- 区域分类（平面/焊缝） ----------\n');
regionInfo = cell(regionCount, 1);  % 存储区域详细信息

for i = 1:regionCount
    regionMask = (regions == i);
    regionPoints = Data(regionMask, :);
    regionCurvatures = curvatures(regionMask);
    regionNormals = normals(regionMask, :);
    
    % 计算区域特征
    avgCurvature = mean(regionCurvatures);
    stdCurvature = std(regionCurvatures);
    numRegionPoints = sum(regionMask);
    
    % 计算法向量一致性（标准差）
    normalStd = std(regionNormals);
    normalConsistency = mean(normalStd);
    
    % 如果使用自动分类方式，则基于曲率和法向量进行分类
    if classifyMethod ~= 2
        % 分类规则：基于平均曲率和法向量一致性
        if avgCurvature < curvatureThreshold * 0.5 && normalConsistency < 0.1
            regionTypes(i) = "平面";
        else
            regionTypes(i) = "焊缝";
        end
    end
    % 如果是手动分类（方式2），regionTypes已经在前面设置好了
    
    % 保存区域信息
    regionInfo{i} = struct(...
        'id', i, ...
        'type', regionTypes(i), ...
        'numPoints', numRegionPoints, ...
        'avgCurvature', avgCurvature, ...
        'stdCurvature', stdCurvature, ...
        'normalConsistency', normalConsistency);
    
    fprintf('  区域 %d: %s | 点数=%d | 平均曲率=%.6f | 法向量一致性=%.6f\n', ...
        i, regionTypes(i), numRegionPoints, avgCurvature, normalConsistency);
end

%% 可视化结果
fprintf('\n---------- 生成可视化 ----------\n');

% 图1：原始点云
figure('Name', '区域生长分割结果', 'Position', [50, 50, 1600, 800]);

subplot(2, 3, 1);
pcshow(pc);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
axis equal;

% 图2：曲率分布
subplot(2, 3, 2);
scatter3(Data(:,1), Data(:,2), Data(:,3), 10, curvatures, 'filled');
colorbar;
title('曲率分布');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on;
axis equal;
colormap(gca, 'jet');

% 图3：区域分割结果（按区域着色）
subplot(2, 3, 3);
hold on;
grid on;
axis equal;
colors = lines(regionCount);  % 生成不同颜色
for i = 1:regionCount
    regionMask = (regions == i);
    regionPoints = Data(regionMask, :);
    scatter3(regionPoints(:,1), regionPoints(:,2), regionPoints(:,3), 10, ...
        colors(i, :), 'filled');
end
% 绘制未分类点（黑色）
unclassified = (regions == 0);
if any(unclassified)
    scatter3(Data(unclassified, 1), Data(unclassified, 2), Data(unclassified, 3), ...
        5, [0.5 0.5 0.5], 'filled');
end
title(sprintf('区域分割结果 (%d个区域)', regionCount));
xlabel('X'); ylabel('Y'); zlabel('Z');

% 图4：平面区域
subplot(2, 3, 4);
hold on;
grid on;
axis equal;
planeCount = 0;
for i = 1:regionCount
    if regionTypes(i) == "平面"
        regionMask = (regions == i);
        regionPoints = Data(regionMask, :);
        scatter3(regionPoints(:,1), regionPoints(:,2), regionPoints(:,3), 10, ...
            colors(i, :), 'filled');
        planeCount = planeCount + 1;
    end
end
title(sprintf('平面区域 (%d个)', planeCount));
xlabel('X'); ylabel('Y'); zlabel('Z');

% 图5：焊缝区域
subplot(2, 3, 5);
hold on;
grid on;
axis equal;
weldCount = 0;
for i = 1:regionCount
    if regionTypes(i) == "焊缝"
        regionMask = (regions == i);
        regionPoints = Data(regionMask, :);
        scatter3(regionPoints(:,1), regionPoints(:,2), regionPoints(:,3), 10, ...
            colors(i, :), 'filled');
        weldCount = weldCount + 1;
    end
end
title(sprintf('焊缝区域 (%d个)', weldCount));
xlabel('X'); ylabel('Y'); zlabel('Z');

% 图6：混合显示（平面=蓝色，焊缝=红色）
subplot(2, 3, 6);
hold on;
grid on;
axis equal;
for i = 1:regionCount
    regionMask = (regions == i);
    regionPoints = Data(regionMask, :);
    if regionTypes(i) == "平面"
        scatter3(regionPoints(:,1), regionPoints(:,2), regionPoints(:,3), 10, ...
            [0 0 1], 'filled');  % 蓝色
    else
        scatter3(regionPoints(:,1), regionPoints(:,2), regionPoints(:,3), 10, ...
            [1 0 0], 'filled');  % 红色
    end
end
title('平面(蓝) vs 焊缝(红)');
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('平面', '焊缝');

%% 保存结果
fprintf('\n是否保存分割结果？\n');
userChoice = input('输入 y/Y 保存，输入 n/N 不保存: ', 's');

if strcmpi(userChoice, 'y')
    % 选择保存路径
    outputPath = uigetdir(pathName, '选择保存分割结果的文件夹');
    if outputPath == 0
        fprintf('未选择保存路径，取消保存！\n');
    else
        [~, baseName, ~] = fileparts(fileName);
        
        % 保存每个区域为单独的点云文件
        fprintf('\n开始保存分割结果...\n');
        for i = 1:regionCount
            regionMask = (regions == i);
            regionPoints = Data(regionMask, :);
            regionNormals = normals(regionMask, :);
            
            % 创建点云对象
            regionPC = pointCloud(regionPoints, 'Normal', regionNormals);
            
            % 根据类型设置不同的颜色
            if regionTypes(i) == "平面"
                % 平面区域：蓝色
                regionColor = repmat([0, 0, 255], size(regionPoints, 1), 1);
            else
                % 焊缝区域：红色
                regionColor = repmat([255, 0, 0], size(regionPoints, 1), 1);
            end
            regionPC.Color = uint8(regionColor);
            
            % 保存文件
            outputFileName = fullfile(outputPath, sprintf('%s_region_%d_%s.ply', ...
                baseName, i, regionTypes(i)));
            pcwrite(regionPC, outputFileName);
            fprintf('  已保存: 区域 %d (%s) -> %s\n', i, regionTypes(i), outputFileName);
        end
        
        % 保存分割信息到文本文件
        infoFileName = fullfile(outputPath, sprintf('%s_segmentation_info.txt', baseName));
        fid = fopen(infoFileName, 'w');
        fprintf(fid, '========== 区域生长分割结果 ==========\n\n');
        fprintf(fid, '输入文件: %s\n', fileName);
        fprintf(fid, '总点数: %d\n', numPoints);
        fprintf(fid, '区域总数: %d\n\n', regionCount);
        fprintf(fid, '参数设置:\n');
        fprintf(fid, '  法向量邻近点数: %d\n', kNeighbors);
        fprintf(fid, '  法向量角度阈值: %.2f 度\n', normalAngleThreshold);
        fprintf(fid, '  曲率阈值: %.4f\n', curvatureThreshold);
        fprintf(fid, '  搜索半径: %.4f\n', searchRadius);
        fprintf(fid, '  最小区域点数: %d\n', minRegionSize);
        fprintf(fid, '  分类方式: %d (1=自动分类, 2=手动前N个)\n', classifyMethod);
        if classifyMethod == 2
            fprintf(fid, '  平面区域数量: 前%d个点数最多的区域\n\n', nPlaneRegions);
        else
            fprintf(fid, '\n');
        end
        fprintf(fid, '区域详细信息:\n');
        fprintf(fid, '%-8s %-10s %-12s %-18s %-18s\n', '区域ID', '类型', '点数', '平均曲率', '法向量一致性');
        fprintf(fid, '%s\n', repmat('-', 1, 80));
        for i = 1:regionCount
            info = regionInfo{i};
            fprintf(fid, '%-8d %-10s %-12d %-18.6f %-18.6f\n', ...
                info.id, info.type, info.numPoints, info.avgCurvature, info.normalConsistency);
        end
        fclose(fid);
        fprintf('  已保存: 分割信息 -> %s\n', infoFileName);
        
        % 保存混合点云（平面和焊缝用不同颜色）
        allColors = zeros(numPoints, 3);
        for i = 1:regionCount
            regionMask = (regions == i);
            if regionTypes(i) == "平面"
                allColors(regionMask, :) = repmat([0, 0, 255], sum(regionMask), 1);  % 蓝色
            else
                allColors(regionMask, :) = repmat([255, 0, 0], sum(regionMask), 1);  % 红色
            end
        end
        % 未分类点为灰色
        unclassified = (regions == 0);
        allColors(unclassified, :) = repmat([128, 128, 128], sum(unclassified), 1);
        
        combinedPC = pointCloud(Data, 'Color', uint8(allColors));
        combinedFileName = fullfile(outputPath, sprintf('%s_segmented_colored.ply', baseName));
        pcwrite(combinedPC, combinedFileName);
        fprintf('  已保存: 混合彩色点云 -> %s\n', combinedFileName);
        
        fprintf('\n保存完成！\n');
    end
else
    fprintf('已取消保存。\n');
end

fprintf('\n========== 程序运行完成 ==========\n');

