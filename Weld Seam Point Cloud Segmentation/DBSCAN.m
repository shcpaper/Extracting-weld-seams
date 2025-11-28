%DBSCAN聚类函数
%data：点云数据
%radius：搜索半径
%MinPts：最小点数
function clusters=DBSCAN(data,radius,MinPts)    
    n = size(data,1);
    kdtree = KDTreeSearcher(data);
    clusters = zeros(n,1);     
    nearPointIndexs = rangesearch(kdtree,data,radius);
 
    %找出所有符合条件的核心对象
    kenelObj = [];
    for i=1:size(nearPointIndexs,1)
        nearPoints = nearPointIndexs{i};
        if length(nearPoints) >= MinPts
            kenelObj(end+1) = i;
        else
            clusters(i) = -1;
        end
    end
 
    %根据密度进行聚类,-1为噪声点,0代表未定义
    classes = 0;
    for i=1:length(kenelObj)
        if clusters(kenelObj(i)) ~= 0
            continue;
        end
 
        result = kenelObj(i);
        classes = classes+1;
        clusters(kenelObj(i)) = classes;
        pointer = 1;
        fprintf("第%d类开始进行聚类...\n",classes);
        %通过广度遍历的方式来进行聚类
        while(pointer <= length(result))
            index = result(pointer);
            
            nearPoints = nearPointIndexs{index};
            if(length(nearPoints) > MinPts)
                tmp = clusters(nearPoints) ~= classes;
                tmpRes = nearPoints(tmp);
                clusters(tmpRes) = classes;
                result=[result,tmpRes];
            end
            pointer = pointer + 1;
        end
    end
end