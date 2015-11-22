function I = rgb2gray(X)
%RGB2GRAY Convert RGB image or colormap to grayscale.
%   RGB2GRAY converts RGB images to grayscale by eliminating the
%   hue and saturation information while retaining the
%   luminance.
%
%   I = RGB2GRAY(RGB) converts the truecolor image RGB to the
%   grayscale intensity image I.
%
%   NEWMAP = RGB2GRAY(MAP) returns a grayscale colormap
%   equivalent to MAP.
%
%   Class Support
%   -------------  
%   If the input is an RGB image, it can be uint8, uint16, double, or
%   single. The output image I has the same class as the input image. If the
%   input is a colormap, the input and output colormaps are both of class
%   double.
%
%   Notes
%   -----
%   RGB2GRAY converts RGB values to grayscale values by forming a weighted 
%   sum of the R, G, and B components:
%
%   0.2989 * R + 0.5870 * G + 0.1140 * B
%
%   The coefficients used to calculate grayscale values in RGB2GRAY are 
%   identical to those used to calculate luminance (E'y) in 
%   Rec.ITU-R BT.601-7 after rounding to 3 decimal places.
%
%   Rec.ITU-R BT.601-7 calculates E'y using the following formula: 
%
%   0.299 * R + 0.587 * G + 0.114 * B 
%
%   Example
%   -------
%   I = imread('example.tif');
%
%   J = rgb2gray(I);
%   figure, imshow(I), figure, imshow(J);
%
%   indImage = load('clown');
%   gmap = rgb2gray(indImage.map);
%   figure, imshow(indImage.X,indImage.map), figure, imshow(indImage.X,gmap);
%
%   See also RGB2IND.

%   Copyright 1992-2015 The MathWorks, Inc.

narginchk(1,1);

[X, threeD] = parse_inputs(X);

if threeD
  I = images.internal.rgb2graymex(X);
else
  % Calculate transformation matrix
  T    = inv([1.0 0.956 0.621; 1.0 -0.272 -0.647; 1.0 -1.106 1.703]);
  coef = T(1,:);
  I = X * coef';
  I = min(max(I,0),1);
  I = [I,I,I];
end


%%%
%Parse Inputs
%%%
function [X, threeD] = parse_inputs(X)
threeD = (ndims(X)==3);
if (threeD)
    if ((size(X,3) ~= 3))
      error(message('MATLAB:images:rgb2gray:invalidInputSizeRGB'))
    end
elseif ismatrix(X)
  if (size(X,2) ~=3 || size(X,1) < 1)
    error(message('MATLAB:images:rgb2gray:invalidSizeForColormap'))
  end
  if ~isa(X,'double')
    error(message('MATLAB:images:rgb2gray:notAValidColormap'))
  end
else
  error(message('MATLAB:images:rgb2gray:invalidInputSize'))
end
%no non-numeric arrays
if ~isnumeric(X)
  error(message('MATLAB:images:rgb2gray:invalidType'))
end
