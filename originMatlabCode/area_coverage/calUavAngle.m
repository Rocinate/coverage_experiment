function uavAngle = calUavAngle(uavControl)
uavNum = size(uavControl,1);
% uavAngle = asin(uavControl(:,2)./(ones(uavNum,1)*vmax));
uavAngle = zeros(uavNum,1);
for i = 1:uavNum
    if sum(uavControl(i,:)) == 0
        uavAngle(i) = 0;
    else
        if uavControl(i,1) >= 0
            uavAngle(i) = asin(uavControl(i,2)/norm(uavControl(i,1:2)));
        elseif uavControl(i,1) < 0 && uavControl(i,2) > 0
            uavAngle(i) = pi - asin(uavControl(i,2)/norm(uavControl(i,1:2)));
        elseif uavControl(i,1) < 0 && uavControl(i,2) < 0
            uavAngle(i) = -pi + asin(-uavControl(i,2)/norm(uavControl(i,1:2)));
        else
            uavAngle(i) = -pi;
        end
    end
end
end