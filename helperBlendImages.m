function outputImage = helperBlendImages(I1, I2) %#codegen
% Identify the image regions in the two images by masking out the black
    % regions.
    maskA = sum(I1, 3) ~= 0;
    maskB = sum(I2, 3) ~= 0;
    maskAB = maskA & maskB;
    
    % Compute alpha values that are proportional to the center seam of the two
    % images.
    alpha1 = ones(size(maskA,1:2));
    alpha2 = ones(size(maskB,1:2));
    dist1  = bwdist(edge(maskA));
    dist2  = bwdist(edge(maskB));

    % 以下为cuixingxing新添加的,添加过渡带渐变区域
%     minScalar = 0.9;
%     maxScalar = 1.1;
%     maskA(maskAB)  = dist1(maskAB) >= maxScalar*dist2(maskAB);
%     maskB(maskAB) = dist1(maskAB) <= minScalar*dist2(maskAB);
%     cond1 = dist1(maskAB) > minScalar*dist2(maskAB);
%     cond2 = dist1(maskAB) < maxScalar*dist2(maskAB);
%     maskAB(maskAB) = cond1&cond2;

    % alpha1,alpha2
    alpha1(maskAB) = dist1(maskAB)./(dist1(maskAB)+dist2(maskAB));
%     alpha1(maskB) = 0;

    alpha2(maskAB) = 1-alpha1(maskAB);
%     alpha2(maskA) = 0;

    I1 = im2double(I1);
    I2 = im2double(I2);        
    outputImage = alpha1.*I1 + alpha2.*I2;    
%     outputImage = im2uint8(outputImage);
end