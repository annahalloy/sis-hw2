%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: Analyze image with MATLAB FFT
%   Last modified: 2021-11-18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear all
close all

%% Load image - Question 2.1
image_name = 'images/example_image.jpg';

% Read the image using imread
data = imread(image_name);
% Convert into gray image
pic = ...
% display image using imshow



%% Prepare for FFT
pixelRows = pic(1,:); % modify this according to your strategy
pixelCols = pic(:,1); % modify this according to your strategy


%% Take fft of rows and columns
Frows = fft(pixelRows,2^nextpow2(length(pixelRows)));
Fmag_rows = real(Frows).^2+imag(Frows).^2;
Fmag_rows_plot = fftshift(Fmag_rows);
x_rows = (0:length(Fmag_rows)-1) - floor(length(Fmag_rows)/2);

Fcols = fft(pixelCols,2^nextpow2(length(pixelCols)));
Fmag_cols = real(Fcols).^2+imag(Fcols).^2;
Fmag_cols_plot = fftshift(Fmag_cols);
x_cols = (0:length(Fmag_cols)-1) - floor(length(Fmag_cols)/2);


%% Analyze image - Question 2.2
black=0; % should be toggled to 1 if black is detected
vertical=0; % should be toggled to 1 if vertical is detected
horizontal=0; % should be toggled to 1 if horizontal is detected

%****************** your image analysis *********************%








%************************************************************%




%% Plots
figure(1)
subplot 321
imshow(data, [0 255])
title('org E-puck picture');

subplot 322
imshow(pic, [0 255])
title('Extracted picture');

subplot 323
plot(pixelRows)
title('Vector of pixels (rows)');
xlabel('pixels');
ylabel('Value');
grid on

subplot 324
plot(pixelCols)
title('Vector of pixels (columns)');
xlabel('pixels');
ylabel('Value');
grid on

subplot 325
plot(x_rows, Fmag_rows_plot)
title('matlab fft rows');
xlabel('Freq');
ylabel('Amplitude');
grid on

subplot 326
plot(x_cols, Fmag_cols_plot)
title('matlab fft columns');
xlabel('Freq');
ylabel('Amplitude');
grid on

if black==1
sgtitle('Detected as black') 
elseif horizontal==1
sgtitle('Detected as horizontal') 
elseif vertical==1
sgtitle('Detected as vertical') 
end