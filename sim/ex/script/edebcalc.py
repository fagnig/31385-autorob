import math

xcw = 0.2
ycw = 0.2
xccw = 0.12
yccw = -0.12
L = 1.0 #sidelength
wb = 0.26 #wheelbase

beta1 = (xcw - xccw)/(-4*L);
beta2 = (ycw + yccw)/(-4*L);
beta = (beta1 + beta2)/2;

Ed = (L + wb/2*beta)/(L - wb/2*beta);

alpha1 = (xcw + xccw)/(-4*L);
alpha2 = (ycw - yccw)/(-4*L);
alpha = (alpha1 + alpha2)/2;

Eb = (math.pi/2)/(math.pi/2 - alpha);

print("Ed =", Ed)
print("Eb =", Eb)