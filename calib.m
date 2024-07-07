%% 利用已知的畸变表格得到opencv鱼眼模型的4个畸变系数，具体请参阅：https://zhuanlan.zhihu.com/p/655174655
distortionTablePath = "./data/AT106K-DISTORTION.xlsx";
sensorRatio = 0.003;% 由厂家提供，单位 mm/pixel

cameraData = readtable(distortionTablePath,Range="A3:C977");
cameraData.Properties.VariableNames = ["theta","refH","realH"];
head(cameraData)% 预览前面若干行数据

angleIn = cameraData.theta;% 入射角
focal = mean(cameraData.refH./tand(angleIn));% 焦距，单位:mm

r_d = 1./focal*cameraData.realH;% 求归一化平面上的r_d
thetaRadian = deg2rad(angleIn);% 度数转为弧度

A = [thetaRadian.^3,thetaRadian.^5,thetaRadian.^7,thetaRadian.^9];
b = r_d-thetaRadian;
opencvCoeffs = A\b;
disp("最小二乘拟合OpenCV鱼眼模型畸变系数为(k1~k4)："+strjoin(string(opencvCoeffs'),","))



%% call opencv
fx = focal./sensorRatio;
fy = focal./sensorRatio;
H = 800;
W = 1280;
intrinsicK = [fx,0,W/2;
    0,fy,H/2;
    0,0,1];
% mapXY = py.cv2.fisheye.initUndistortRectifyMap(intrinsicK,opencvCoeffs',eye(3),...
%     intrinsicK,[W,H],py.cv2.CV_32FC1);

newCameraMatrixK = intrinsicK;
distortFrame = imread("./data/front.bmp");
% newImageSize = size(distortFrame,[1,2]);% [height,width]
newImageSize = [800,3000];% [height,width]
[mapX,mapY] = initUndistortRectifyMapOpenCV(intrinsicK, opencvCoeffs,newCameraMatrixK,newImageSize);

undistortImg2 = images.internal.interp2d(distortFrame,mapX,mapY,"linear",255, false);
figure;
imshow(undistortImg2)
title("undistortion image from fit opencv fisheye model coefficient")
