clear all
close all
clc

lpFilt = designfilt('lowpassiir','FilterOrder',2, ...
    'PassbandFrequency',10,'PassbandRipple',0.003, ...
         'SampleRate',1e3);

fvtool(lpFilt)
t = 0:.001:1;
x = ones(size(t)); % Create sawtooth signal.
data_in = awgn(x,50,'measured'); % Add white Gaussian noise.




dataOut = filter(lpFilt,data_in);
mean_data=mean(data_in);
dev=std(data_in);

C= lpFilt.Coefficients;

[b,a] = tf(lpFilt);
disp('Coef a:');vpa(a,5)

disp('Coef b:');vpa(b,5)


figure(2)
plot(t,x,t,data_in,t,dataOut) % Plot both signals.
legend('Original signal','Signal with AWGN','Signal Filtered');
grid on



lpFilt = designfilt('lowpassiir','FilterOrder',2, ...
    'PassbandFrequency',5,'PassbandRipple',0.003, ...
         'SampleRate',1e3);

fvtool(lpFilt)
t = 0:.001:1;
x = ones(size(t)); % Create sawtooth signal.
data_in = awgn(x,50,'measured'); % Add white Gaussian noise.




dataOut = filter(lpFilt,data_in);
mean_data=mean(data_in);
dev=std(data_in);

C= lpFilt.Coefficients;

[b,a] = tf(lpFilt);
disp('Coef a:');vpa(a,5)

disp('Coef b:');vpa(b,5)


figure(4)
plot(t,x,t,data_in,t,dataOut) % Plot both signals.
legend('Original signal','Signal with AWGN','Signal Filtered');
grid on