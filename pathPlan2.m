function path = pathPlan2(mapData,startPose,goalPose)%#codegen
% Brief: costmap全局路径规划
% Details:
%    None
%
% Syntax:
%     path = pathPlan(mapData,startPose,goalPose)
%
% Inputs:
%    mapData - [m,n] size,[numerical] type,costmap地图
%    startPose - [1，3] size,[double] type,起始位置，[x,y,theta]，x,y单位为米，theta单位为弧度
%    goalPose - [1，3] size,[double] type,终止位置，[x,y,theta]，x,y单位为米，theta单位为弧度
%    resolution - [1,1] size,[double] type,代表cells/米
%    originInWorld - [1,2] size,[double] type,像素地图的原点在世界地图的位置
%
% Outputs:
%    path - [N,3] size,[double] type,[x,y,theta]，x,y单位为米，theta单位为弧度
%
% Example:
%    None
%
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         18-Feb-2024 14:02:28
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024a
% Copyright © 2024 TheMatrix.All Rights Reserved.
%
arguments
    mapData (:,:)  {mustBeInRange(mapData,0,1)}
    startPose (1,3) double
    goalPose (1,3) double
    resolution (1,1) double
    originInWorld (1,2) double
end
threshold = 0.1;
inflatRadius = 1.5; % 单位：meter
inflatRadiusPixel = round(inflatRadius*resolution);
bw = mapData>threshold;
se = strel("disk",inflatRadiusPixel);
bw = imdilate(bw,se);
map = binaryOccupancyMap(bw,resolution);
map.LocalOriginInWorld = originInWorld;

stateSpace = stateSpaceSE2;
stateSpace.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
validator = validatorOccupancyMap(stateSpace,Map=map);
validator.ValidationDistance = 0.01;

planner = plannerHybridAStar(validator);
pathObj = plan(planner,startPose,goalPose);
path = pathObj.States;
end