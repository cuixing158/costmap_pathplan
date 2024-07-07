function [mapX,mapY,undistortPts, distortPts] = initUndistortRectifyMapOpenCV(K, opencvCoeffs,newCameraMatrixK,newImageSize)
% Brief: 由opencv鱼眼畸变系数得到映射和坐标点对应，功能等同于opencv的initUndistortRectifyMap函数
% Details:
%    None
%
% Syntax:
%     [mapX,mapY,undistortPts, distortPts] = initUndistortRectifyMapOpenCV(w, h, K, opencvCoeffs)
%
% Inputs:
%    K - [3,3] size,[double] type,fisheye camera intrinsic,
%       [fx,0,cx;
%       0,fy,cy;
%       0,0,1] format
%    opencvCoeffs - [1,4] size,[double] type,opencv fisheye coeffs
%    newCameraMatrixK - [3,3] size,[double] type,new fisheye camera intrinsic,
%       [fx,0,cx;
%       0,fy,cy;
%       0,0,1] format
%    newImageSize - [1,2],[double] type,new image [height,width]
%
% Outputs:
%    mapX - [h,w] size,[double] type,Description
%    mapY - [h,w] size,[double] type,Description
%    undistortPts - [h*w,2] size,[double] type,Description
%    distortPts - [h*w,2] size,[double] type,Description
%
% Example:
%   oriImg = imread("fishEye.png");
%
%   % from opencv fisheye camera calibration results
%   opencvCoeffs = [-0.01078350003808737,0.04842806980013847,-0.04542399942874908,0.008737384341657162];  % mapping polynomial coefficients,[k1,k2,p1,p2]
%   [h,w,c] = size(oriImg);
%   K = [286.7037963867188, 0, 413.3463134765625;
%       0, 286.7817993164062, 397.1785888671875;
%       0, 0, 1];
%
%   [mapX,mapY] = initUndistortRectifyMapOpenCV(K,opencvCoeffs,K,[h,w]);
%   distortImg = im2double(distortImg);
%   undistortImg = images.internal.interp2d(distortImg,mapX,mapY,"linear",0, false);
%   figure;
%   imshow(undistortImg)
%
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         27-Sep-2022 09:42:05
% Version history revision notes:
%                        2022-11-23修改为支持自定义输入新内参矩阵和输出图像大小指定。
%                        2023-09-06修改变量命名，theta_d改为r_d,修改数值计算稳定性。
%                        2024-01-23修改以畸变中心为输出的正确尺寸大小。
% Note:
%     1.Implement paper "opencv fisheye model by Juho Kannala and Sami Brandt."
%     2.入射角theta,畸变半径r_d都是针对单位成像平面计算的，最终目的得到scale,与实际成像平面成正比。
%
% Implementation In Matlab R2022a
% Copyright © 2022 TheMatrix.All Rights Reserved.
%
arguments
    K (3,3) {mustBeNumeric}
    opencvCoeffs (1,4) {mustBeNumeric}
    newCameraMatrixK (3,3) {mustBeNumeric}
    newImageSize (1,2) {mustBeNumeric}
end

% coeff convert to matlab
cx = round(newImageSize(2)/2);
cy = round(newImageSize(1)/2);
c0 = K(1,3);
c1 = K(2,3);
[undistortX,undistortY] = meshgrid(1-cx+c0:newImageSize(2)-cx+c0,1-cy+c1:newImageSize(1)-cy+c1);
undistortPts = [undistortX(:),undistortY(:)];

undistortPtsHomo = [undistortPts';
    ones(1,prod(newImageSize))]; % 3*cols size
undistortCameraPts = newCameraMatrixK\undistortPtsHomo; % 3*cols size
undistortCameraPts = undistortCameraPts./undistortCameraPts(end,:);% 3*cols size

r = vecnorm(undistortCameraPts(1:2,:),2,1); % 1*cols size
theta = atan(r);

r_d = theta.*(1+opencvCoeffs(1)*theta.^2+opencvCoeffs(2)*theta.^4+...
    opencvCoeffs(3)*theta.^6+opencvCoeffs(4)*theta.^8); % r_d非theta_d

r(r<=10^(-8))=1;
scale =r_d./r;
u = K(1,1)*undistortCameraPts(1,:).*scale+ K(1,3);
v = K(2,2)*undistortCameraPts(2,:).*scale + K(2,3);
distortPts = [u',v'];% rows*2

mapX = reshape(distortPts(:,1),newImageSize(1),newImageSize(2));
mapY = reshape(distortPts(:,2),newImageSize(1),newImageSize(2));
end