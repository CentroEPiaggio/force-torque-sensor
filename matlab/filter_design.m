%% Calcualte the coefficient (numerator and denominator  [b, a]) of the filter for the force torque sensor filter.

clear all
close all
clc

%% Design the filter as a Low Pass Filter of a Second Order and Pass Band of 10 Hz
lpFilt = designfilt('lowpassiir','FilterOrder',2, ...
    'PassbandFrequency',10,'PassbandRipple',0.003, ...
         'SampleRate',1e3);
% show filter Filter Visualitazione Tool
fvtool(lpFilt)

t = 0:.001:1; % time vector
x = ones(size(t)); % Create a step signal signal.
data_in = awgn(x,50,'measured'); % Add white Gaussian noise.


% Test filter on data simulated 
dataOut = filter(lpFilt,data_in);

% Returns filter parameters
[b,a] = tf(lpFilt);

%Show filter parameters
disp('Coef a:');vpa(a,5)
disp('Coef b:');vpa(b,5)

% Plot noise and filtered signal  
figure(2)
plot(t,x,t,data_in,t,dataOut) 
legend('Original signal','Signal with AWGN','Signal Filtered');
grid on

%% Design the filter as a Low Pass Filter of a Second Order and Pass Band of 10 Hz

lpFilt = designfilt('lowpassiir','FilterOrder',2, ...
    'PassbandFrequency',5,'PassbandRipple',0.003, ...
         'SampleRate',1e3);
% show filter Filter Visualitazione Tool
fvtool(lpFilt)
% Test filter on data simulated 
dataOut = filter(lpFilt,data_in);

% Returns filter parameters
[b,a] = tf(lpFilt);
disp('Coef a:');vpa(a,5)
disp('Coef b:');vpa(b,5)

% Plot noise and filtered signal  
figure(4)
plot(t,x,t,data_in,t,dataOut) % Plot both signals.
legend('Original signal','Signal with AWGN','Signal Filtered');
grid on