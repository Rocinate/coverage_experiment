function NewNewData = DataTransform(FsyData,Range)
%数据转化，将大维度数据转换为小维度
%输入：原数据：FsyData； 转换后范围：Range
%输出：小维度数据：NewNewData
number = size(FsyData,1);
narrow = number/Range;
for k =1:Range
    NewData(:,k) = sum(FsyData(:,(k-1)*narrow+1:k*narrow),2)/narrow;
end
for k =1:Range
    NewNewData(k,:) = sum(NewData((k-1)*narrow+1:k*narrow,:),1)/narrow;
end
end