function bev = makeBEV(frontImg,leftImg,backImg,rightImg)
% Brief: 从四副鱼眼环视图拼接为俯视图
% Details:
%    None
%
% Syntax:
%     bev = makeBEV(frontImg,leftImg,backImg,rightImg)
%
% Inputs:
%    frontImg - [m,n] size,[double] type,Description
%    leftImg - [m,n] size,[double] type,Description
%    backImg - [m,n] size,[double] type,Description
%    rightImg - [m,n] size,[double] type,Description
%
% Outputs:
%    bev - [m,n] size,[double] type,Description
%
% Example:
%    None
%
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         23-Jan-2024 15:18:51
% Version history revision notes:
%                                  None
% Implementation In Matlab R2024a
% Copyright © 2024 TheMatrix.All Rights Reserved.
%
persistent mapX mapY birdsEye tforms
if isempty(mapX)
    load data/birdsEye360.mat % 数据来源于本项目中的getCameraParames.m
end

BEV = cell(1,4);

undistortImage = interp2d(frontImg,mapX,mapY,"linear",255, false);
BEV{1} = transformImage(birdsEye{1},undistortImage);

undistortImage = interp2d(leftImg,mapX,mapY,"linear",255, false);
BEV{2} = transformImage(birdsEye{2},undistortImage);

undistortImage = interp2d(backImg,mapX,mapY,"linear",255, false);
BEV{3} = transformImage(birdsEye{3},undistortImage);

undistortImage = interp2d(rightImg,mapX,mapY,"linear",255, false);
BEV{4} = transformImage(birdsEye{4},undistortImage);

images = BEV;
tt = {tforms(1),tforms(2),tforms(3),tforms(4)};
[bev, outputView] = helperStitchImages(images,tt);

end
