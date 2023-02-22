% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Simple Pendulum Control Part III

legfnt = 13;

% Pendulum parameters
disp('> Simple pendulum parameters')
m = 0.5
l = 0.3
r = l/2
b = 0
g = 9.8
J = m*l^2/12
Jr = (J+m*r^2)

% models linearized around equilibria
disp('> Linearized models')
Gpi = tf(1/Jr, [1, b/Jr, -m*r*g/Jr])
G0 = tf(1/Jr, [1, b/Jr, m*r*g/Jr])
wn = 7;

% model for uncertainty analysis
a1 = b/Jr
a2 = m*g*r/Jr
b2 = 1/Jr
F = tf(1, [1 a1 0])

% C6 controller
disp('> Controller C6')
ctrC6 = zpk([-6.855, -0.9573], [0, -21], 3.8321)

% C7 controlller
z1 = 1;
z2 = 5;
pc = 11;
ctrC7 = tf(conv([1 z1],[1 z2]),conv([1 0],[1 pc]));
K = 2/abs(freqresp(ctrC7,wn));

disp('> Controller C7')
ctrC7 = zpk(K * ctrC7)

% C8 controller
disp('> Controller C8')
z1 = 1;
z2 = 3;
pc = 10.5;
ctrC8 = tf(conv([1 z1],[1 z2]),conv([1 0],[1 pc]))
K = 2/abs(freqresp(ctrC8,wn));
ctrC8 = zpk(K * ctrC8)

% Compare sensitivities

disp('> Sensitivity function (S6)')
S6 = feedback(1,Gpi*ctrC6)
zpk(S6)

disp('> Damping (S6)')
damp(S6)

disp('> Sensitivity function (S7)')
S7 = feedback(1,Gpi*ctrC7)
zpk(S7)

disp('> Damping (S7)')
damp(S7)

disp('> Sensitivity function (S8)')
S8 = feedback(1,Gpi*ctrC8)
zpk(S8)

disp('> Damping (S8)')
damp(S8)

L6 = Gpi*ctrC6;
L7 = Gpi*ctrC7;
L8 = Gpi*ctrC8;

% Bode plots

xl = [-1 2];

[magS7,phsS7,w] = bode(S7, {10^xl(1), 10^xl(2)});
[magS6,phsS6] = bode(S6, w);
[magS8,phsS8] = bode(S8, w);

[magL6,phsL6] = bode(L6, w);
[magL7,phsL7] = bode(L7, w);
[magL8,phsL8] = bode(L8, w);

[magC6,phsC6] = bode(ctrC6, w);
[magC7,phsC7] = bode(ctrC7, w);
[magC8,phsC8] = bode(ctrC8, w);

% Fix phases
phsL6 = phsL6 + 360;
phsL7 = phsL7 + 360;
phsL8 = phsL8 + 360;

yl1 = [-30 20];
yl2 = [0 270];

% Fig. 8.2:

figure(1), clf

hline = semilogx(w, 20*log10([magS6(:)'; magS7(:)'; ...
                    magL6(:)'; magL7(:)']));
grid on

xlim(10.^xl);
ylim(yl1);

set(gca, 'YTick', [-30 -20 -10 0 10 20]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$S_6$', '$S_7$', '$L_6$', '$L_7$', ...
           'Location', 'NorthWest'); 
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');

figure(2), clf

hline = semilogx(w, ([phsS6(:)'; phsS7(:)'; ...
                    phsL6(:)'; phsL7(:)']));
grid on

xlim(10.^xl);
ylim(yl2);

set(gca, 'YTick', [0 90 180 270 360]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$S_6$', '$S_7$', '$L_6$', '$L_7$', ...
           'Location', 'NorthWest'); 
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (dB)');
xlabel('\omega (rad/s)');

% Fig. 8.2:

yl1 = [0 20];
yl2 = [-30 20];

figure(3), clf

hline = semilogx(w, 20*log10([magC6(:)'; magC7(:)'; magC8(:)']));
grid on

xlim(10.^xl);
ylim(yl1);

%set(gca, 'YTick', []);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$C_6$', '$C_7$', '$C_8$', ...
           'Location', 'SouthWest'); 
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude');
xlabel('\omega (rad/s)');

figure(4), clf

hline = semilogx(w, 20*log10([magL6(:)'; magL7(:)'; magL8(:)']));
grid on

xlim(10.^xl);
ylim(yl2);

%set(gca, 'YTick', [0 90 180 270 360]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_6$', '$L_7$', '$L_8$', ...
           'Location', 'SouthWest'); 
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude');
xlabel('\omega (rad/s)');


figure(5), clf

yl2 = [90 225];

hline = semilogx(w, ([phsL6(:)'; phsL7(:)'; phsL8(:)']));
grid on

xlim(10.^xl);
ylim(yl2);

set(gca, 'YTick', [0 90 180 270 360]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_6$', '$L_7$', '$L_8$', ...
           'Location', 'NorthWest'); 
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (dB)');
xlabel('\omega (rad/s)');


% Hoo analysis

disp('> G6')
G6 = -a2*feedback(F,b2*ctrC6)
disp('> |G6|oo')
norm(G6,inf)

disp('> G7')
G7 = -a2*feedback(F,b2*ctrC7)
disp('> |G7|oo')
norm(G7,inf)

disp('> G8')
G8 = -a2*feedback(F,b2*ctrC8)
disp('> |G8|oo')
norm(G8,inf)

% Bode plots

xl = [-1 1.8];
yl = [0 1.2];

[magG7,phsG7,w] = bode(G7, {10^xl(1), 10^xl(2)});
[magG6,phsG6] = bode(G6, w);
[magG8,phsG8] = bode(G8, w);

figure(6), clf

semilogx(w, [magG6(:)'; magG7(:)'; magG8(:)'], ...
         [w(1) w(end)], [1 1], 'k--');
grid on

xlim(10.^xl);
ylim(yl);

set(gca, 'YTick', [0 0.5 1]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$G_6$', '$G_7$', '$G_8$', 'Location', 'NorthWest');
set(l, 'interpreter', 'latex', 'FontSize', 15);
ylabel('Magnitude');
xlabel('\omega (rad/s)');


% Circle criterion

% g6

eps0 = 0.02;
offset = 0.1;

[re,im] = nyquist(G6, {eps0,1e2});
b6 = re(:) + j*im(:);

% g7

eps0 = 0.02;
offset = 0.1;

[re,im] = nyquist(G7, {eps0,1e2});
b7 = re(:) + j*im(:);

% g8

eps0 = 0.1;
offset = 0.1;

[re,im] = nyquist(G8, {eps0,1e2});
b8 = re(:) + j*im(:);

% circles

t = 0 : 0.1 : 2*pi;
circ1 = cos(t) + j*sin(t);

alpha = -0.22;
beta = 1;
cc = -(1/alpha+1/beta)/2;
rr = abs(-1/alpha+1/beta)/2;
circ2 = cc + rr*(cos(t) + j*sin(t));

figure(7), clf

xl = [-1.25 4.75];
yl = 1.2*[-1.25 1.25];

plot(real([b6; b6]), imag([b6; -b6]), ...
     real([b7; b7]), imag([b7; -b7]), ...
     real([b8; b8]), imag([b8; -b8]))

h = rectangle('Position',[-1,-1,2,2], 'Curvature',[1,1], ...
          'FaceColor', 0.1*[1 0 0] + .9*[1 1 1]);
uistack(h, 'bottom')
h = rectangle('Position',[-1,-rr,2*rr,2*rr], 'Curvature',[1,1], ...
              'FaceColor', 0.1*[0 1 0] + .9*[1 1 1]);
uistack(h, 'bottom')

l = legend('$G_6$', '$G_7$', '$G_8$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);

xlim(xl)
ylim(yl)

xlabel('Re');
ylabel('Im');

grid on
set(gca, 'layer', 'top')

pbaspect([1 diff(ylim)/diff(xlim) 1]);


% Damping study

bs = linspace(0, 0.5, 100);
Gs = zeros(3,length(bs));

i = 1;
for b = bs

  % model for uncertainty feedback
  a1 = b/Jr;
  a2 = m*g*r/Jr;
  b2 = 1/Jr;
  F = tf(1, [1 a1 0]);

  G6 = -a2*feedback(F,b2*ctrC6);
  G7 = -a2*feedback(F,b2*ctrC7);
  G8 = -a2*feedback(F,b2*ctrC8);
  Gs(:,i) = [norm(G6,inf); norm(G7,inf); norm(G8,inf)];

  i = i + 1;

end

xl = [bs(1) bs(end)];
yl = [.25 1.25];

figure(8), clf
plot(bs, Gs, ...
     xl, [1 1], 'k--');
grid on

xlim(xl);
ylim(yl);

set(gca, 'YTick', [.25 0.5 0.75 1 1.25]);
set(gca, 'XTick', [0 0.1 0.2 0.3 0.4 0.5]);

set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$G_6$', '$G_7$', '$G_8$', 'Location', 'NorthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('||G||_\infty');
xlabel('b');
