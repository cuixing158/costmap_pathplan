%% 此脚本功能用于直接估计获取相机内外参+俯视拼接图转换矩阵，省去标定过程
% 参考我个人项目semanticNet/preprocessData.mlx中的文件改写

calibImgs = ["front.bmp","left.bmp","rear.bmp","right.bmp"];% pay attention to order
imgRootDir = "./data/";
imgLists = fullfile(imgRootDir,calibImgs);
numImgs = numel(imgLists);

distortionImage = imread(imgLists(1));
[H,W,~] = size(distortionImage);

% [xCenter,yCenter,lambda] = estCameraParamForDivisionModel(M);
xCenter = W/2; % 经多次实验估计，应当使用此值
yCenter = H/2;
lambda = -2.5817e-06;

% 拟合的系数,参阅本项目的calib.m估计
K = 1.0e+02 *[3.178183050207416,0 ,6.400000000000000;
    0,3.178183050207416, 4.000000000000000;
    0,0,0.010000000000000];
opencvCoeffs = [0.114531912975723
    -0.031552081308369
    0.010706989306028
    -0.002092521007301];% k1~k4 for opencv fisheye camera
imageSize = [800,3000];% [height,width] pixels
[mapX,mapY] = initUndistortRectifyMapOpenCV(K, opencvCoeffs,K,imageSize);

distortImages = cell(1,numImgs);
undistortImages = cell(1,numImgs);
BEV = cell(1,numImgs);
birdsEye = cell(1,numImgs);
for i = 1:numImgs
    distortionImage = imread(imgLists(i));
    distortImages{i} = distortionImage;
    % 自定义输出图像范围ROI,此值为以原始图像distortionImage的几何/畸变中心为原点的像素坐标系下的坐标！
    % ROI = [-1500,-400,3000,800]; % 形如[x,y,width,height],单位:像素
    % [undistortImages{i},camIntrinsic,mapX,mapY] = undistortImageForDivisionModel(distortionImage,...
    %     lambda,xCenter,yCenter,OutputViewROI=ROI);
    undistortImages{i} = images.internal.interp2d(distortionImage,mapX,mapY,"linear",255, false);
end
% figure;imshow(undistortImages{4});
% title("undistorted image")

%% 估计外参
% Front,使用drawpolygon分别交互式得到trapezoid
trapezoid = 1.0e+03*[1.057355080270725   0.344657678087815
    0.077500000000000   0.707500000000000
    2.877500000000000   0.707500000000000
    1.936441109179120   0.345103126012491];
% figure;imshow(undistortImages{1});
% roi = drawpolygon(Position=trapezoid,FaceAlpha=0.2,LineWidth=1);%预览
rectangleWidth = 6.185308;%单位：米，根据已知chessBoardDimension.png尺寸说明
rectangleLength = 1.8;%单位：米

[pitch,yaw,roll,height,intrinsicsOut] = estimateMonoCameraFromScene(trapezoid,...
    rectangleWidth,rectangleLength,imageSize);

% 预览前视畸变投影为BEV效果图
monoSensor = monoCamera(intrinsicsOut,height,Pitch=pitch,Yaw=yaw,Roll=roll);

outView = [0,10,-10,10];
outImageSize = [480,NaN];
birdsEye{1} = birdsEyeView(monoSensor,outView,outImageSize);
BEV{1} = transformImage(birdsEye{1},undistortImages{1});
% figure;imshow(BEV{1});title("BEV")

%% Left,使用drawpolygon分别交互式得到trapezoid
trapezoid = 1.0e+03*[0.872683320536280   0.327303906836965
    0.124274035977638   0.621509361433806
    2.647500000000000   0.619500000000000
    2.016184447508481   0.324298647633358];
% figure;imshow(undistortImages{2});
% roi = drawpolygon(Position=trapezoid,FaceAlpha=0.2,LineWidth=1);%预览
rectangleWidth = 8.4;%单位：米，根据已知的chessBoardDimension.png说明
rectangleLength = 1.892654;%单位：米
[pitch,yaw,roll,height,intrinsicsOut] = estimateMonoCameraFromScene(trapezoid,...
    rectangleWidth,rectangleLength,imageSize);

% 预览left视畸变投影为BEV效果图
monoSensor = monoCamera(intrinsicsOut,height,Pitch=pitch,Yaw=yaw,Roll=roll);

% 注意，这里是以当前视角看的,每行形如[xmin,xmax,ymin,ymax]，其坐标值是指在世界自车坐标系（汽车坐标系）中自定义可视化区域
outView = [0,10,-10,10];
outImageSize = [480,NaN];
birdsEye{2} = birdsEyeView(monoSensor,outView,outImageSize);
BEV{2} = transformImage(birdsEye{2},undistortImages{2});
% figure;imshow(BEV{2});title("BEV")

%% Back,使用drawpolygon分别交互式得到trapezoid
trapezoid = 1.0e+03*[1.118067095805827   0.415947646601716
    0.317102356093016   0.700656938390525
    2.675500000000000   0.701500000000000
    1.888009536399179   0.417076594461824];
% figure;imshow(undistortImages{3});
% roi = drawpolygon(Position=trapezoid,FaceAlpha=0.2,LineWidth=1);%预览
rectangleWidth = 6.185308;%单位：米，根据已知的chessBoardDimension.png尺寸说明
rectangleLength = 1.8;%单位：米
[pitch,yaw,roll,height,intrinsicsOut] = estimateMonoCameraFromScene(trapezoid,...
    rectangleWidth,rectangleLength,imageSize);

% 预览left视畸变投影为BEV效果图
monoSensor = monoCamera(intrinsicsOut,height,Pitch=pitch,Yaw=yaw,Roll=roll);

% 注意，这里是以当前视角看的,每行形如[xmin,xmax,ymin,ymax]，其坐标值是指在世界自车坐标系（汽车坐标系）中自定义可视化区域
outView = [0,10,-10,10];
outImageSize = [480,NaN];
birdsEye{3} = birdsEyeView(monoSensor,outView,outImageSize);
BEV{3} = transformImage(birdsEye{3},undistortImages{3});
% figure;imshow(BEV{3});title("BEV")

%% Right,使用drawpolygon分别交互式得到trapezoid
trapezoid = 1.0e+03*[0.982509184516607   0.323548279090795
    0.357500000000000   0.619500000000000
    2.883500000000000   0.621500000000000
    2.131564799500170   0.324173094171189];
% figure;imshow(undistortImages{4});
% roi = drawpolygon(Position=trapezoid,FaceAlpha=0.2,LineWidth=1);%预览
rectangleWidth = 8.4;%单位：米，根据已知的chessBoardDimension.png尺寸说明
rectangleLength = 1.892654;%单位：米
[pitch,yaw,roll,height,intrinsicsOut] = estimateMonoCameraFromScene(trapezoid,...
    rectangleWidth,rectangleLength,imageSize);

% 预览left视畸变投影为BEV效果图
monoSensor = monoCamera(intrinsicsOut,height,Pitch=pitch,Yaw=yaw,Roll=roll);

% 注意，这里是以当前视角看的,每行形如[xmin,xmax,ymin,ymax]，其坐标值是指在世界自车坐标系（汽车坐标系）中自定义可视化区域
outView = [0,10,-10,10];
outImageSize = [480,NaN];% 自定义图像宽度或者高度，其另一边自动推算，这个依赖指定的outVIew值
birdsEye{4} = birdsEyeView(monoSensor,outView,outImageSize);
BEV{4} = transformImage(birdsEye{4},undistortImages{4});% 鸟瞰图
% figure;imshow(BEV{4});title("BEV")

showImg = imtile([distortImages,undistortImages,BEV],GridSize=[3,4]);
positions = linspace(1,size(showImg,2),numImgs*2+1);
positions = [positions(2:2:end)',20*ones(numImgs,1)];
showImg = insertText(showImg,positions,calibImgs,FontSize=50);
figure;imshow(showImg)
title("4副鱼眼环视BEV图像分别投影")

%% 分别计算front->front,left->front,back->front,right->front的2D刚性转换矩阵tform
isSelectControlPoints = false;
if isSelectControlPoints
    tforms = repmat(simtform2d(),1,numImgs);
    if isfile("data/birdsEye360.mat")
        load data/birdsEye360.mat
        % left->front
        [selectedMovingPoints1,selectedFixedPoints1] = cpselect(BEV{2},BEV{1},...
            selectedMovingPoints1,selectedFixedPoints1,'Wait',true);
        tforms(2) = fitgeotform2d(selectedMovingPoints1,selectedFixedPoints1,"similarity");
        
        % back->front
        [selectedMovingPoints2,selectedFixedPoints2] = cpselect(BEV{3},BEV{2},...
            selectedMovingPoints2,selectedFixedPoints2,'Wait',true);
        temp = fitgeotform2d(selectedMovingPoints2,selectedFixedPoints2,"similarity");
        tforms(3).A = tforms(2).A*temp.A;
        
        % right->front
        [selectedMovingPoints3,selectedFixedPoints3] = cpselect(BEV{4},BEV{1},...
            selectedMovingPoints3,selectedFixedPoints3,'Wait',true);
        tforms(4) = fitgeotform2d(selectedMovingPoints3,selectedFixedPoints3,"similarity");
        
        % fine-tune refine back->front tforms(3)
        [selectedMovingPoints4,selectedFixedPoints4] = cpselect(BEV{3},BEV{4},...
            selectedMovingPoints4,selectedFixedPoints4,'Wait',true);
        tform34 = fitgeotform2d(selectedMovingPoints4,selectedFixedPoints4,"similarity");
        tform31 = simtform2d(tforms(4).A*tform34.A);
    else
        % left->front
        [selectedMovingPoints1,selectedFixedPoints1] = cpselect(BEV{2},BEV{1},'Wait',true);
        tforms(2) = fitgeotform2d(selectedMovingPoints1,selectedFixedPoints1,"similarity");
        
        % back->front
        [selectedMovingPoints2,selectedFixedPoints2] = cpselect(BEV{3},BEV{2},'Wait',true);
        temp = fitgeotform2d(selectedMovingPoints2,selectedFixedPoints2,"similarity");
        tforms(3).A = tforms(2).A*temp.A;
        
        % right->front
        [selectedMovingPoints3,selectedFixedPoints3] = cpselect(BEV{4},BEV{1},'Wait',true);
        tforms(4) = fitgeotform2d(selectedMovingPoints3,selectedFixedPoints3,"similarity");
        
        % fine-tune refine back->front tforms(3)
        [selectedMovingPoints4,selectedFixedPoints4] = cpselect(BEV{3},BEV{4},'Wait',true);
        tform34 = fitgeotform2d(selectedMovingPoints4,selectedFixedPoints4,"similarity");
        tform31 = simtform2d(tforms(4).A*tform34.A);
    end
    
    scale = mean([tforms(3).Scale,tform31.Scale]);
    theta1 = tforms(3).RotationAngle;
    theta2 = tform31.RotationAngle;
    if theta1<0
        theta1 = theta1+360;
    end
    if theta2<0
        theta2 = theta2+360;
    end
    theta = mean([theta1,theta2]);
    t = mean([tforms(3).Translation;
        tform31.Translation]);
    tforms(3) = simtform2d(scale,theta,t);
    
    % 保存4个鱼眼相机各类参数,后续可直接拿任意同一时刻四张鱼眼图像做环视拼接
    save("data/birdsEye360.mat","mapX","mapY","birdsEye","tforms",...
        "selectedMovingPoints1","selectedFixedPoints1",...
        "selectedMovingPoints2","selectedFixedPoints2",...
        "selectedMovingPoints3","selectedFixedPoints3",...
        "selectedMovingPoints4","selectedFixedPoints4");
    
else
    load data/birdsEye360.mat
end

% 展示初始4张图像拼接效果
images = BEV;
tt = {tforms(1),tforms(2),tforms(3),tforms(4)};
[outputImage, outputView] = helperStitchImages(images,tt);
figure;imshow(outputImage,outputView)
title("拼接的360环视图")