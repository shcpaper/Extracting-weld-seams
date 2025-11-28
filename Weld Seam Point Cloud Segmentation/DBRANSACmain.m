%DBSCAN聚类方法（基于法向量）
clc
clear
close all;
 
%获取点云数据
[fileName,pathName]=uigetfile({'*.pcd';'*.txt';'*.ply'},'Input Data-File');   %选择要进行计算的三维点云数据文件路径
 
if isempty(fileName) || length(fileName) == 1
    fprintf("未选择点云文件！\n");
    return;
end
pc = pcread([pathName,fileName]);
Data = pc.Location;         %获取点云的位置信息

%计算点云法向量
fprintf("正在计算点云法向量...\n");
tic
normals = pcnormals(pc, 10);  %使用10个邻近点计算法向量
toc
fprintf("法向量计算完成！\n");

%使用法向量进行DBSCAN聚类
fprintf("开始基于法向量的DBSCAN聚类...\n");
tic
clusters = DBSCAN(normals, 0.5, 10000);  %使用法向量数据进行聚类
classes = unique(clusters);
toc
fprintf("共有%d个类别（包含噪声点）\n",length(classes));
 
%显示聚类结果（使用原始点云位置）
figure
hold on
grid on
rotate3d on
axis equal
for i=1:length(classes)
    clusterRes = Data(clusters==classes(i),:);
    plot3(clusterRes(:,1),clusterRes(:,2),clusterRes(:,3),'.');
end
title('基于法向量的DBSCAN聚类');
xlabel('X');
ylabel('Y');
zlabel('Z');

%显示法向量方向（可选，显示部分法向量）
figure
pcshow(pc);
hold on
%每隔10个点显示一个法向量，避免显示过于密集
step = 10;
quiver3(Data(1:step:end,1), Data(1:step:end,2), Data(1:step:end,3), ...
        normals(1:step:end,1), normals(1:step:end,2), normals(1:step:end,3), ...
        0.5, 'r', 'LineWidth', 1);
title('点云及其法向量');
xlabel('X');
ylabel('Y');
zlabel('Z');

%询问用户是否保存聚类结果
fprintf('\n是否将聚类结果保存为点云文件？\n');
userChoice = input('输入 y/Y 保存，输入 n/N 不保存: ', 's');

if strcmpi(userChoice, 'y')
    %选择输出格式
    fprintf('\n请选择输出格式：\n');
    fprintf('1 - PLY格式\n');
    fprintf('2 - PCD格式\n');
    fprintf('3 - TXT格式\n');
    formatChoice = input('请输入选项(1/2/3): ', 's');
    
    switch formatChoice
        case '1'
            fileExt = '.ply';
        case '2'
            fileExt = '.pcd';
        case '3'
            fileExt = '.txt';
        otherwise
            fprintf('输入无效，默认使用PLY格式\n');
            fileExt = '.ply';
    end
    
    %选择保存路径
    outputPath = uigetdir(pathName, '选择保存聚类结果的文件夹');
    if outputPath == 0
        fprintf('未选择保存路径，取消保存！\n');
    else
        %保存每个类别的点云
        fprintf('\n开始保存聚类结果...\n');
        savedCount = 0;
        for i=1:length(classes)
            if classes(i) == -1
                %跳过噪声点，或者可以选择保存噪声点
                fprintf('跳过噪声点（类别-1）\n');
                continue;
            end
            
            %获取该类别的点云数据
            clusterIndices = (clusters == classes(i));
            clusterData = Data(clusterIndices, :);
            
            %如果该类别有法向量信息，也保存
            if ~isempty(normals)
                clusterNormals = normals(clusterIndices, :);
                clusterPC = pointCloud(clusterData, 'Normal', clusterNormals);
            else
                clusterPC = pointCloud(clusterData);
            end
            
            %生成文件名
            [~, baseName, ~] = fileparts(fileName);
            outputFileName = fullfile(outputPath, sprintf('%s_cluster_%d%s', baseName, classes(i), fileExt));
            
            %根据格式保存文件
            try
                if strcmp(fileExt, '.txt')
                    %TXT格式：保存为纯文本，每行为 x y z
                    writematrix(clusterData, outputFileName, 'Delimiter', ' ');
                else
                    %PLY或PCD格式
                    pcwrite(clusterPC, outputFileName);
                end
                savedCount = savedCount + 1;
                fprintf('已保存类别 %d，共 %d 个点 -> %s\n', classes(i), size(clusterData, 1), outputFileName);
            catch ME
                fprintf('保存类别 %d 时出错: %s\n', classes(i), ME.message);
            end
        end
        fprintf('\n保存完成！共保存了 %d 个聚类结果文件。\n', savedCount);
    end
else
    fprintf('已取消保存。\n');
end