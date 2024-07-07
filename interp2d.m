function outputImage = interp2d(inputImage,X,Y,method,fillValues, varargin)%#codegen
% FOR INTERNAL USE ONLY -- This function is intentionally
% undocumented and is intended for use only within other toolbox
% classes and functions. Its behavior may change, or the feature
% itself may be removed in a future release.
%
% Vq = INTERP2D(V,XINTRINSIC,YINTRINSIC,METHOD,FILLVAL, SmoothEdges)
% computes 2-D interpolation on the input grid V at locations in the
% intrinsic coordinate system XINTRINSIC, YINTRINSIC. The value of the
% output grid Vq(I,J) is determined by performing 2-D interpolation at
% locations specified by the corresponding grid locations in
% XINTRINSIC(I,J), YINTRINSIC(I,J). XINTRINSIC and YINTRINSIC are plaid
% matrices of the form constructed by MESHGRID. When V has more than two
% dimensions, the output Vq is determined by interpolating V a slice at a
% time beginning at the 3rd dimension.
%
% See also INTERP2, MAKERESAMPLER, MESHGRID

% Copyright 2012-2018 The MathWorks, Inc.

% Algorithm Notes
%
% This function is intentionally very similar to the MATLAB INTERP2
% function. The differences between INTERP2 and images.internal.interp2d
% are:
%
% 1) Edge behavior. This function uses the 'fill' pad method described in
% the help for makeresampler. When the interpolation kernel partially
% extends beyond the grid, the output value is determined by blending fill
% values and input grid values.
% This behavior is on by default, unless SmoothEdges is specified and set
% to false.
%
% 2) Plane at a time behavior. When the input grid has more than 2 
% dimensions, this function treats the input grid as a stack of 2-D interpolation
% problems beginning at the 3rd dimension.
%
% 3) Degenerate 2-D grid behavior. Unlike interp2, this function handles
% input grids that are 1-by-N or N-by-1.

%#ok<*EMCA>

narginchk(5,6);
if(nargin==5)    
    % If not specified, default to NOT smoothing edges
    SmoothEdges = false;
else
    SmoothEdges = varargin{1};
end

if ~coder.target('MATLAB')
    coder.inline('always');
    coder.internal.prefer_const(inputImage,X,Y,method,fillValues);
    outputImage = images.internal.coder.interp2d(inputImage,X,Y,method,fillValues, SmoothEdges);
    return;
end

fillValues = cast(fillValues, 'like', inputImage);

if(SmoothEdges)
    [inputImage,X,Y] = padImage(inputImage,X,Y,fillValues);
end

if isreal(inputImage)    
    [outputImage, XYHasNaNs] = imagesbuiltinImageInterpolation2D(inputImage,X,Y,method,fillValues);    
else    
    [outputImageR, XYHasNaNsR] = imagesbuiltinImageInterpolation2D(real(inputImage),X,Y,method,real(fillValues));
    [outputImageI, XYHasNaNsI] = imagesbuiltinImageInterpolation2D(imag(inputImage),X,Y,method,imag(fillValues));
    XYHasNaNs = XYHasNaNsR|| XYHasNaNsI;
    outputImage = complex(outputImageR, outputImageI);
end

if(XYHasNaNs)
    validateattributes(X, {'numeric'}, {'nonnan'}, mfilename, 'X', 2);
    validateattributes(Y, {'numeric'}, {'nonnan'}, mfilename, 'Y', 3);
end

function [paddedImage,X,Y] = padImage(inputImage,X,Y,fillValues)
% We achieve the 'fill' pad behavior from makeresampler by prepadding our
% image with the fillValues and translating our X,Y locations to the
% corresponding locations in the padded image. We pad two elements in each
% dimension to account for the limiting case of bicubic interpolation,
% which has a interpolation kernel half-width of 2.

pad = 3;
X = X+pad;
Y = Y+pad;

sizeInputImage = size(inputImage);
sizeOutputImage = sizeInputImage;
sizeOutputImage(1:2) = sizeOutputImage(1:2) + [2*pad 2*pad];

if isscalar(fillValues)
    paddedImage = zeros(sizeOutputImage,'like',inputImage);
    paddedImage(:) = fillValues;
    if(ismatrix(inputImage))
        paddedImage(4:end-3,4:end-3,:) = inputImage;
    else
        for pInd = 1:prod(sizeOutputImage(3:end))
            paddedImage(4:end-3,4:end-3,pInd) = inputImage(:,:,pInd);
        end
    end

else
    paddedImage = zeros(sizeOutputImage,'like',inputImage);
    [~,~,numPlanes] = size(inputImage);
    for i = 1:numPlanes
        paddedImage(:,:,i) = padarray(inputImage(:,:,i),[pad pad],fillValues(i));
    end
end
