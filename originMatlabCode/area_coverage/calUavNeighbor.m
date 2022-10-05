function uavNeighbor = calUavNeighbor(A,uavId)
uavNeighbor = zeros(size(A,1),10);
for i=1:size(A,1)
    uavIndex=find(A(i,:)>0);%邻居集索引（不包括自己）
    if length(uavIndex) > 10
        uavNeighbor(i,:) = uavId(uavIndex(1:10));
    else
        uavNeighbor(i,:) = [uavId(uavIndex)' zeros(1,10-length(uavIndex))]; 
    end
end
end