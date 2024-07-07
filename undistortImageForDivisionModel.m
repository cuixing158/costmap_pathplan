function [J,camIntrinsic,mapX,mapY,newOrigin] = undistortImageForDivisionModel(I,lambda,x_center,y_center,options)
% Brief: 使用除法模型distortion division model已知的三个参数(lambda,x_center,y_center)对畸变图像I进行去畸变，适用略微和严重畸变类型。
% Details:
%    对畸变图像I使用distortion division model中包含一阶畸变系数lambda和畸变
% 中心x_center,y_center进行去畸变，得对应camIntrinsic的无畸变图J。
%
% Syntax:
%     [J,camIntrinsic,mapX,mapY] = undistortImageForDivisionModel(I,lambda,x_center,y_center)
%
% Inputs:
%    I - [m,n] size,any image type,畸变图像
%    lambda - [1,1] size,[double] type,畸变系数
%    x_center - [1,1] size,[double] type,畸变中心横坐标
%    y_center - [1,1] size,[double] type,畸变中心纵坐标
%    options.OutputView和options.OutputViewROI必须仅有一个参数输入，其中
% OutputViewROI参数输入形如[x,y,width,height],前2个值代表输出图像undistortImg
% 的(1,1)坐标在原图像I上几何/畸变中心为原点的像素坐标。
%
% Outputs:
%    J - [M,N] size,any image type,去畸变图像
%    camIntrinsic - [1,1] size,[cameraIntrinsics] type,build-in cameraIntrinsics (https://ww2.mathworks.cn/help/vision/ref/cameraintrinsics.html)
%    mapX - [M,N] size,[double] type,映射X坐标
%    mapY - [M,N] size,[double] type,映射Y坐标
%    newOrigin - [1,2] size, [double] type,代表输出图像undistortImg的(1,1)坐
% 标在原图像I上几何/畸变中心为原点的像素坐标。
%
% References:
%   https://www.wikiwand.com/en/Distortion_(optics)
%
% Example:
%    None
%
% See also: None
% Release Note:
% 2023.12.5
% 1、valid模式和full模式只适用于入射角小于90度的情况，如果有大于90度
%               况，则应当指定NewOrigin输入参数。

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         13-Jun-2023 08:46:34
% Version history revision notes:
%                                  None
% Implementation In Matlab R2023b
% Copyright © 2023 TheMatrix.All Rights Reserved.
%

arguments
    I
    lambda (1,1) double
    x_center (1,1) double
    y_center (1,1) double
    options.OutputView (1,:) char {mustBeMember(options.OutputView,["full","valid","same"])}
    options.OutputViewROI (1,4) double
end
flag1 = isfield(options,"OutputView");
flag2 = isfield(options,"OutputViewROI");
if (flag1&&flag2)|| (~flag1&&~flag2)
    error("options:OutputView and NewOrigin must only occur once");
end

[H,W,~] = size(I);
% recovery undistorted image,参考维基百科Distortion_(optics)的reverse-distortion公式
if flag1
    if strcmpi(options.OutputView,"same")
        newOrigin = -[x_center,y_center];
        [x_u,y_u] = meshgrid(1:W,1:H);
    elseif strcmpi(options.OutputView,"full")
        xEdges = [1:W,W*ones(1,H),1:W,ones(1,H)];
        yEdges = [ones(1,W),1:H,H*ones(1,W),1:H];
        conersEdgePts = [xEdges(:),yEdges(:)];
        x_d = conersEdgePts(:,1);
        y_d = conersEdgePts(:,2);
        r = sqrt((x_d-x_center).^2+(y_d-y_center).^2);
        x_u = x_center+(x_d-x_center)./(1+lambda*r.^2);
        y_u = y_center+(y_d-y_center)./(1+lambda*r.^2);
        [minA,maxA] = bounds([x_u,y_u]);
        newOrigin = [minA(1),minA(2)]-[x_center,y_center];
        [x_u,y_u] = meshgrid(minA(1):maxA(1),minA(2):maxA(2));
    else
        % 参考来源：本项目中的undistortFisheyeFromTable/undistortFisheyeImgFromTable.m实现
        
        % 求不规则四条曲线边轮廓的最大内接矩形
        xEdges = [1:W,W*ones(1,H),W:-1:1,ones(1,H)]';% top,right,down,left edge order
        yEdges = [ones(1,W),1:H,H*ones(1,W),H:-1:1]';
        conersEdgePts = [xEdges(:),yEdges(:)];
        x_d = conersEdgePts(:,1);
        y_d = conersEdgePts(:,2);
        r = sqrt((x_d-x_center).^2+(y_d-y_center).^2);
        x_u = x_center+(x_d-x_center)./(1+lambda*r.^2);
        y_u = y_center+(y_d-y_center)./(1+lambda*r.^2);
        undistortPts = [x_u,y_u];
        
        topEdgePoints = undistortPts(1:W,:);
        rightEdgePoints = undistortPts(W+1:W+H,:);
        bottomEdgePoints = undistortPts(W+H+1:2*W+H,:);
        leftEdgePoints = undistortPts(2*W+H+1:end,:);
        
        xmin = max(leftEdgePoints(:,1));
        xmax = min(rightEdgePoints(:,1));
        ymin = max(topEdgePoints(:,2));
        ymax = min(bottomEdgePoints(:,2));
        assert((xmin<xmax)&&(ymin<ymax),"Invalid parameter lambda, too small or too large a value");
        newOrigin = [xmin,ymin]-[x_center,y_center];
        [x_u,y_u] = meshgrid(xmin:xmax,ymin:ymax);
    end
else
    x0 = options.OutputViewROI(1);
    y0 = options.OutputViewROI(2);
    width = options.OutputViewROI(3);
    height = options.OutputViewROI(4);
    
    xx = (x0:x0+width-1)+x_center;
    yy = (y0:y0+height-1)+y_center;
    newOrigin = [x0,y0];
    [x_u,y_u] = meshgrid(xx,yy);
end

r_u = sqrt((x_u-x_center).^2+(y_u-y_center).^2);
x_d = x_center+(x_u-x_center).*(1-sqrt(1-4*lambda.*r_u.^2))./(2*lambda.*r_u.^2);
y_d = y_center+(y_u-y_center).*(1-sqrt(1-4*lambda.*r_u.^2))./(2*lambda.*r_u.^2);

mapX = fillmissing(x_d,"constant",1);
mapY = fillmissing(y_d,"constant",1);

J = images.internal.interp2d(I,mapX,mapY,...
    "linear",0, false);

% 以下等价无畸变图像内参矩阵,其计算参考官方undistortFisheyeImage函数源码
imageSize = size(J,[1,2]);
f = min(size(I,[1,2]))/2;
principalPoint = -newOrigin + 0.5;
K = [f,0,principalPoint(1);
    0,f,principalPoint(2);
    0,0,1];
camIntrinsic = cameraIntrinsics([K(1,1),K(2,2)],principalPoint,imageSize);
end

function mustBeExclusive(options)
flag1 = isfield(options,"OutputView");
flag2 = isfield(options,"NewOrigin");
if (flag1&&flag2)|| (~flag1&&~flag2)
    error("options:OutputView and NewOrigin only occur once");
end
end


