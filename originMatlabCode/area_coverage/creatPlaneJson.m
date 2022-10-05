function s = creatPlaneJson(uavPos)
uavNum = size(uavPos,1);
uav = cell(uavNum,1);
for i = 1:uavNum
    uav{i}.pos = [100*uavPos(i,:) 1000];
    uav{i}.angle = 0;
    uav{i}.id = i;
end
data.Uav = uav;
s.messageID = 1000;
s.data = data;
end