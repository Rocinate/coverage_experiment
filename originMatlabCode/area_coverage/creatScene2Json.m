function s = creatScene2Json(uavPos,uavControl,A,cOPI,v,c,curves,dt)
uavNum = size(uavPos,1);
uavNeighbor = calUavNeighbor(A,(1:uavNum)');
uav = cell(uavNum,1);
uavAngle = calUavAngle(uavControl);
for i = 1:uavNum
    uav{i}.pos = [100*uavPos(i,:) 1000];
    uav{i}.angle = uavAngle(i)*180/pi;
    uav{i}.id = i;
    uav{i}.speed = 100*norm(uavControl(i,:))*dt;
    uav{i}.neighborId = uavNeighbor(i,:);
    uav{i}.centroid = 100*cOPI(i,:);
    uav{i}.voronoi_x = 100*v(c{i},1)';
    uav{i}.voronoi_y = 100*v(c{i},2)';
end
data.Uav = uav;
data.curves = curves;
s.messageID = 1002;
s.data = data;
end