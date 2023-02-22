% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Simple Pendulum Control Part II

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

disp('> Open-loop dynamics')
den = Gpi.den{1};
wn = sqrt(-den(3)/den(1))
zeta = den(2)/(2*wn*den(1))

disp('> Loop transfer-function with integrator')
L0 = G0 * tf(1,[1 0])
Lpi = Gpi * tf(1,[1 0])

% Fig. 7.22:

xl = [-1 2];
yl1 = [-80 40];
yl2 = [-360 180];

[magL0,phsL0,w] = bode(L0, {10^xl(1), 10^xl(2)});
ind = find(abs(w - wn) < 1e-2);
phsL0(ind(1)+1) = nan;

[magLpi,phsLpi] = bode(Lpi, w);

% Fix phase
phsL0 = phsL0;
phsLpi = phsLpi+360;

figure(1), clf

semilogx(w, 20*log10([magL0(:)'; magLpi(:)']));

xlim(10.^xl);
ylim(yl1);
grid on

set(gca, 'YTick', [-80 -40 0 40]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_0$', '$L_\pi$', 'Location', 'SouthWest');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');


figure(2), clf

semilogx(w, ([phsL0(:)'; phsLpi(:)']));

xlim(10.^xl);
ylim(yl2);
grid on 

set(gca, 'YTick', [-360 -270 -180 -90 0 90 180]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_0$', '$L_\pi$', 'Location', 'SouthWest');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (dB)');
xlabel('\omega (rad/s)');


% C6 design

% Root-locus controller
disp('> Controller from root-locus design (C6)')
ctrC6 = zpk([-6.855, -0.9573], [0, -21], 3.8321)

disp('> Loop transfer-function with C6')
Lpi6 = ctrC6*Gpi

[magGpi,phsGpi] = bode(Gpi, w);
[magLpi6,phsLpi6] = bode(Lpi6, w);
[magCtrC6,phsCtrC6] = bode(ctrC6, w);

% Fix phase
phsGpi = phsGpi+360;
phsLpi6 = phsLpi6+360;

% Margins
[gmpi,pmpi,wgmpi,wpmpi] = margin(Lpi6);
Spi = feedback(1,Lpi6);
sm = 1/norm(Spi,inf);

disp('> Gain Margin in dB (G_M)')
20*log10(gmpi)

disp('> Phase Margin in deg (phi_M)')
pmpi

disp('> Stability Margin (S_M)')
sm

gmpi = freqresp(Lpi6,wgmpi);
pmpi = freqresp(Lpi6,wpmpi);

% Fig. 7.23:

xl = [-1 2];
yl1 = [-40 20];
yl2 = [-90 270];

figure(3), clf

semilogx(w, 20*log10([magLpi6(:)'; magGpi(:)'; magCtrC6(:)']), ...
         wgmpi, 20*log10(abs(gmpi)), 'ok')

xlim(10.^xl);
ylim(yl1);
grid on

set(gca, 'YTick', [-40 -20 0 20]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_\pi$', '$G_\pi$', '$C_6$', 'Location', 'SouthWest');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');


figure(4), clf

semilogx(w, ([phsLpi6(:)'; phsGpi(:)'; phsCtrC6(:)']), ...
         wpmpi, 360+180/pi*phase(pmpi), 'ko')

xlim(10.^xl);
ylim(yl2);
grid on

set(gca, 'YTick', [-90 0 90 180 270]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_\pi$', '$G_\pi$', '$C_6$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (dB)');
xlabel('\omega (rad/s)');

% Fig. 7.24:

figure(5)

nyquist(Lpi6, {5e-1,1e2});


% Controller C7

z1 = 1;
z2 = 5;
pc = 11;
ctrC7 = tf(conv([1 z1],[1 z2]),conv([1 0],[1 pc]));
K = 2/abs(freqresp(ctrC7,wn));

disp('> Controller C7')
ctrC7 = zpk(K * ctrC7)

disp('> Loop transfer-function with C7')
Lpi7 = ctrC7*Gpi

[magLpi7,phsLpi7] = bode(Lpi7, w);
[magCtrC7,phsCtrC7] = bode(ctrC7, w);

% Fix phase
phsLpi7 = phsLpi7+360;

% Margins
[gmpi,pmpi,wgmpi,wpmpi] = margin(Lpi7);
Spi = feedback(1,Lpi7);
sm = 1/norm(Spi,inf);

disp('> Gain Margin in dB (G_M)')
20*log10(gmpi)

disp('> Phase Margin in deg (phi_M)')
pmpi

disp('> Stability Margin (S_M)')
sm

gmpi = freqresp(Lpi7,wgmpi);
pmpi = freqresp(Lpi7,wpmpi);


% Bode plots 

xl = [-1 2];
yl1 = [-40 20];
yl2 = [-90 270];

figure(6), clf

semilogx(w, 20*log10([magLpi7(:)'; magGpi(:)'; magCtrC7(:)']), ...
         wgmpi, 20*log10(abs(gmpi)), 'ok')

xlim(10.^xl);
ylim(yl1);
grid on

set(gca, 'YTick', [-40 -20 0 20]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_\pi$', '$G_\pi$', '$C_7$', 'Location', 'SouthWest');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');


figure(7), clf

semilogx(w, ([phsLpi7(:)'; phsGpi(:)'; phsCtrC7(:)']), ...
         wpmpi, 360+180/pi*phase(pmpi), 'ko')

xlim(10.^xl);
ylim(yl2);
grid on

set(gca, 'YTick', [-90 0 90 180 270]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_\pi$', '$G_\pi$', '$C_7$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (dB)');
xlabel('\omega (rad/s)');


% Fig. 7.25: Compare controllers

xl = [-1 2];
yl1 = [-20 20];
yl2 = [-90 90];

[magCtrC6,phsCtrC6] = bode(ctrC6, w);
[magCtrC7,phsCtrC7] = bode(ctrC7, w);

figure(8), clf

[haxes,hline1,hline2] = ...
  plotyy(w, 20*log10([magCtrC6(:)'; magCtrC7(:)']), ...
	 w, [phsCtrC6(:)'; phsCtrC7(:)'], ...
	 'semilogx', 'semilogx');

set(haxes(1), 'XLim', 10.^xl);
set(haxes(2), 'XLim', 10.^xl);
set(haxes(1), 'YLim', yl1);
set(haxes(2), 'YLim', yl2);

set(haxes, 'Box', 'off');
set(haxes, 'YGrid', 'on');
set(haxes, 'XGrid', 'on');

set(haxes(1), 'YTick', [-20 -10 0 10 20]);
set(haxes(2), 'YTick', [-90 -45 0 45 90]);

set(haxes, 'PlotBoxAspectRatio', [1 1/2 1]);

ylabel(haxes(1), 'Magnitude (dB)');
ylabel(haxes(2), 'Phase (deg)');
xlabel(haxes(1), '\omega (rad/s)');

l = legend(hline1, '$C_6$', '$C_7$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);


% G0 with C7

L07 = ctrC7*G0

[magL07,phsL07] = bode(L07, w);
ind = find(abs(w - wn) < 1e-2);
phsL07(ind(1)+1) = nan;

[magG0,phsG0] = bode(G0, w);
ind = find(abs(w - wn) < 1e-2);
phsG0(ind(1)+1) = nan;

% Margins
[gm0,pm0,wgm0,wpm0] = margin(L07);
S0 = feedback(1,L07);
sm = 1/norm(S0,inf);

disp('> Gain Margin in dB (G_M)')
20*log10(gm0)

disp('> Phase Margin in deg (phi_M)')
pm0

disp('> Stability Margin (S_M)')
sm

gm0 = freqresp(L07,wgm0);
pm0 = freqresp(L07,wpm0);


% Bode plots 

xl = [-1 2];
yl1 = [-40 20];
yl2 = [-270 90];

figure(9), clf

semilogx(w, 20*log10([magL07(:)'; magG0(:)'; magCtrC7(:)']), ...
         wgm0, 20*log10(abs(gm0)), 'ok')

xlim(10.^xl);
ylim(yl1);
grid on

set(gca, 'YTick', [-40 -20 0 20]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_0$', '$G_0$', '$C_7$', 'Location', 'SouthWest');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');


figure(10), clf

semilogx(w, ([phsL07(:)'; phsG0(:)'; phsCtrC7(:)']), ...
         wpm0, 180/pi*phase(pm0), 'ko')

xlim(10.^xl);
ylim(yl2);
grid on

set(gca, 'YTick', [-360 -270 -180 -90 0 90]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$L_0$', '$G_0$', '$C_7$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (dB)');
xlabel('\omega (rad/s)');


% Fig. 7.28:

figure(11)

nyquist(L07, {5e-1,6}), hold on
nyquist(L07, {8,1e2}), hold off


% Fig. 8.2: Sensitivities

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

L6 = Gpi*ctrC6;
L7 = Gpi*ctrC7;

[magS7,phsS7,w] = bode(S7, {10^xl(1), 10^xl(2)});
[magS6,phsS6] = bode(S6, w);

[magL6,phsL6] = bode(L6, w);
[magL7,phsL7] = bode(L7, w);

% Fix phases
phsL6 = phsL6 + 360;
phsL7 = phsL7 + 360;

xl = [-1 2];
yl1 = [-30 20];
yl2 = [0 270];

figure(12), clf

hline = semilogx(w, 20*log10([magS6(:)'; magS7(:)'; magL6(:)'; magL7(:)']));
grid on

xlim(10.^xl);
ylim(yl1);

set(gca, 'YTick', [-30 -20 -10 0 10 20]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$S_6$', '$S_7$', '$L_6$', '$L_7$', 'Location', 'NorthWest');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');

figure(13), clf

hline = semilogx(w, ([phsS6(:)'; phsS7(:)'; phsL6(:)'; phsL7(:)']));
grid on

xlim(10.^xl);
ylim(yl2);

set(gca, 'YTick', [0 90 180 270 360]);
set(gca, 'PlotBoxAspectRatio', [1 1/3 1]);

l = legend('$S_6$', '$S_7$', '$L_6$', '$L_7$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (dB)');
xlabel('\omega (rad/s)');
