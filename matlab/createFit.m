function [fitresult, gof] = createFit1(close_vec, w)
%CREATEFIT1(CLOSE_VEC,W)
%  Create a fit.
%
%  Data for 'weight hand fitting' fit:
%      X Input : close_vec
%      Y Output: w
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 15-Feb-2016 12:37:21


%% Fit: 'weight hand fitting'.
[xData, yData] = prepareCurveData( close_vec, w );

% Set up fittype and options.
ft = fittype( 'smoothingspline' );
opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.Normalize = 'on';
opts.SmoothingParam = 0.999999993834772;

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'weight hand fitting' );
h = plot( fitresult, xData, yData );
legend( h, 'w vs. close_vec', 'weight hand fitting', 'Location', 'NorthEast' );
% Label axes
xlabel close_vec
ylabel w
grid on


