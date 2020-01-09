function [Ed, Eb] = robot_calibrator(Xcw, Ycw, Xccw, Yccw, L, b0)
%UNTITLED2 Coming soon

beta1 = (Xcw - Xccw)/(-4*L);
beta2 = (Ycw + Yccw)/(-4*L);
beta = (beta1 + beta2)/2;

%R = (L/2)/sin(beta/2);
%R = (L/2)/(beta/2);

Ed = (L + b0/2*beta)/(L - b0/2*beta);

alpha1 = (Xcw + Xccw)/(-4*L);
alpha2 = (Ycw - Yccw)/(-4*L);
alpha = (alpha1 + alpha2)/2;

%b_actual = (pi/2)/(pi/2 - alpha)*b0;

Eb = (pi/2)/(pi/2 - alpha);

end

