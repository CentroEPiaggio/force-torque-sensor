clear all
close all
clc
close_vec=[0, 0.15, 0.25, 0.56, 0.7, 0.8, 0.9, 0.99];
w=[6.98114, 6.80429, 6.75198, 6.97346, 6.86624, 6.80824, 6.62334, 6.67071];

[fitresult, gof] = createFit(close_vec, w)

breaks=fitresult.p.breaks;
coeff=fitresult.p.coefs;

for i=0:0.01:1
%    0.0440    
%    0.4234    
%    0.6943    
%    0.9653    
   
end