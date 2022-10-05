function [cx,cy] = OptimalPositionNonuniform(Px,Py,Data,gridX,gridY)
%非均匀密度函数计算最优位置
%输入：无人机位置：Px,Py；场强数据：Data；栅格化：gridX,gridY
%输出：最优位置cx，cy
number = size(Px,2);
DataRange = size(gridX,1);
dataX = zeros(DataRange^2,number); %坐标x的分配
dataY = zeros(DataRange^2,number); %坐标y的分配
dataS = zeros(DataRange^2,number); %场强的分配

for j = 1:DataRange %通过计算距离，将点分配给维诺划分
    for k = 1:DataRange
        [minDistan,UAVID]=min(sqrt((Px-(gridX(j,k)+zeros(1,number))).^2+...
            (Py-(gridY(j,k)+zeros(1,number))).^2));
        dataX((j-1)*DataRange+k,UAVID) = gridX(j,k);
        dataY((j-1)*DataRange+k,UAVID) = gridY(j,k);
        dataS((j-1)*DataRange+k,UAVID) = Data(j,k);
    end    
end
%% 质心计算
mVoronoi = sum(dataS);
cx = sum(dataX.*dataS)./mVoronoi;
cy = sum(dataY.*dataS)./mVoronoi;
end