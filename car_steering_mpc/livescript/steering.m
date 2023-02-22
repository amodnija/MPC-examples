A = [   -5  -0.3427;
     47.68    2.785];
B = [    0   1
       0.3   0];
C = flipud(eye(2));
D = zeros(2);
CSTR = ss(A,B,C,D);