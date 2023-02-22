% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Chapter 7

legfnt = 13;

% Fig. 7.5:

w1 = 7.5;

w2 = 21;

K = 3.83;

sys = K * tf([1 w1], [1 w2])

zpk(sys)

figure(20), clf

w = logspace(-1,3,300);
[mag,phs] = bode(sys, w);
[wl,magl,wp,phsl] = basym(sys);

xl = 10.^[0 2];
yl = [0 15];

semilogx(wl, magl, '-', wl,magl, '.', ...
         w, 20*log10([mag(:)']))

text(w1, magl(2)-1, '(A)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(w2, magl(4)+1, '(B)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');

xlim(xl)
ylim(yl)

ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');

% Fig. 7.6:

figure(21), clf

xl = 10.^[-1 3];
yl = [-5 30];

semilogx(wp, phsl,'-', wp, phsl,'.', ...
         w, [phs(:)'])

xlim(xl)
ylim(yl)

text(wp(2), phsl(2)-2, '(A)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(3), phsl(3)+2, '(B)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(4), phsl(4)+2, '(C)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(5), phsl(5)-2, '(D)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');

%set(gca,'YTick',[-360 -315 -270 -225 -180 -135 -90]);

ylabel('Phase (deg)');
xlabel('\omega (rad/s)');

% Fig. 7.5:

w1 = .5;
zeta1 = .1;

w2 = 2;
zeta2 = .2;

w3 = 3;
zeta3 = .1;

w4 = 5;

K = 10;

sys = K * tf([1], [(1/w1)^2 2*zeta1/w1 1]) * tf([(1/w2)^2 2*zeta2/w2 1], [1]) * tf([1], [(1/w3)^2 2*zeta3/w3 1]) * tf([1], [1/w4 1]) * tf([1], [1 0])

zpk(sys)

sys = tf([28.1 22.4 112.4],[1 5.7 12.81 47.6 7.5 11.25 0])

zpk(sys)


figure(1), clf

w = logspace(-2,2,300);
[mag,phs] = bode(sys, w);
[wl,magl,wp,phsl] = basym(sys);

xl = 10.^[-1 1];
yl = [-50 50];

semilogx(wl, magl, '-', wl,magl, '.', ...
         w, 20*log10([mag(:)']))

text(.5, magl(2)-8, '(A)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(2, magl(4)+8, '(B)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(3, magl(6)-8, '(C)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(5, magl(8)+10, '(D)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');

xlim(xl)
ylim(yl)

ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');


% Fig. 7.6:

figure(2), clf

xl = 10.^[-2 2];
yl = [-370 -80];

semilogx(wp, phsl,'-', wp, phsl,'.', ...
         w, [phs(:)'])

xlim(xl)
ylim(yl)

text(wp(2), phsl(2)-20, '(A)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(3), phsl(3)+20, '(B)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(4), phsl(4)-20, '(C)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(5)+.2, phsl(5)+20, '(D)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(6), phsl(6)+25, '(E)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(8)+2, phsl(8)+20, '(G)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(7), phsl(7)+25, '(F)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');
text(wp(9)+10, phsl(9)+20, '(H)', 'BackgroundColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center','interpreter','latex');

set(gca,'YTick',[-360 -315 -270 -225 -180 -135 -90]);

ylabel('Phase (deg)');
xlabel('\omega (rad/s)');


% Non-minimum phase

sys1 = tf([1 1], [1 1 1]);
sys2 = tf([-1 1], [1 1 1]);
sys3 = tf(conv([-1 1],[-1 1]), [1 2 2 1]);

w = logspace(-1,1,100);
[mag1,phs1] = bode(sys1, w);
[mag2,phs2] = bode(sys2, w);
[mag3,phs3] = bode(sys3, w);

phs2 = phs2 - 360;
phs3 = phs3 - 360;

[wl1,magl1,wp1,phsl1] = basym(sys1);
[wl2,magl2,wp2,phsl2] = basym(sys2);
[wl3,magl3,wp3,phsl3] = basym(sys3);


% Fig. 7.7(a)

figure(3)

xl = [-1 1];
yl1 = [-20 10];

semilogx(w, 20*log10([mag1(:)']), ...
         wl1, magl1, '-', ...
         wl1, magl1, '.');

xlim(10.^xl);
ylim(yl1);
grid

set(gca, 'YTick', [-20 -10 0 10]);
set(gca, 'PlotBoxAspectRatio', [1 .275 1]);

l = legend('$G_1, G_2, G_3$~', 'Location', 'NorthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');


% Fig. 7.7(b)

figure(4), clf

yl2 = [-450 0];

h = semilogx(w, phs1(:)', '-', ...
             w, phs2(:), '--', ...
             w, phs3(:)', '-.', ...
             wp1, phsl1, '-', ...
             wp1, phsl1, '.', ...
             wp2, phsl2, '-', ...
             wp2, phsl2, '.', ...
             wp3, phsl3, '-', ...
             wp3, phsl3, '.');

xlim(10.^xl);
ylim(yl2);

set(gca, 'YTick', [-450 -360 -270 -180 -90 0 90 180 270 360]);
set(gca, 'PlotBoxAspectRatio', [1 .275 1]);

grid

l = legend('$G_1$','$G_2$', '$G_3$', 'Location', 'SouthWest');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Phase (deg)');
xlabel('\omega (rad/s)');


% Fig. 7.8: step response

T = 10;
[y1, t1] = step(sys1, T);
[y2, t2] = step(sys2, T);
[y3, t3] = step(sys3, T);

figure(5)

plot(t1, y1, '-', t2, y2, '--', t3, y3, '-.', ...
     [0 T], [1 1], '--k');

xlim([0 T]);
ylim([-0.5 1.5]);
grid
pbaspect([1 1/2 1]);

l = legend('$G_1$', '$G_2$', '$G_3$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
xlabel('t')
ylabel('y(t)')


% Fig. 7.10 (a)

sys = tf(1,[1 1]);

xl = [-1 1];
ylm = [-20 0];
ylp = [-90 0];

[mag,phs,w] = bode(sys, {10^xl(1), 10^xl(2)});
[wl,magl,wp,phsl] = basym(sys);

we = [0 inf];
fe = freqresp(sys,we);

wr = [0.1 1 9.9];
fr = freqresp(sys,wr);

figure(6), clf

semilogx(w, 20*log10(mag(:)'), ...
         wl, magl, '-', ...
         wl, magl, '.', ...
         wr, 20*log10(abs(fr(:))), 'o', ...
         we, 20*log10(abs(fe(:))), 'x');

set(gca,'YTick',[-20 -10 0]);
grid on

xlim(10.^xl);
ylim(ylm);
pbaspect([1 1/4 1]);

ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');

% Fig. 7.10 (b)

figure(7), clf

semilogx(w, phs(:), ...
         wp, phsl, '-', ...
         wp, phsl, '.', ...
         wr, 180*phase(fr(:))/pi, 'o', ...
         we, 180*phase(fe(:))/pi, 'x');

set(gca,'YTick',[-90 -45 0]);
grid on

xlim(10.^xl);
ylim(ylp);

ylabel('Phase (deg)');
xlabel('\omega (rad/s)');

pbaspect([1 1/4 1]);


% Fig. 7.10 (c)

[re,im] = nyquist(sys);

figure(8), clf

xl = [-.2 1.2];
yl = [-.6 .6];

plot(re(:), im(:), '-', ...
     re(:), -im(:), '--', ...
     real(fr(:)), imag(fr(:)), 'o', ...
     real(fe(:)), imag(fe(:)), 'x', ...
     xl, [0 0], '-k', ...
     [0 0], yl, '-k')

xlim(xl)
ylim(yl)

i = floor(7 * length(re) / 16);
h = arrow([re(i), im(i)], [re(i+1), im(i+1)], ...
          'TipAngle', 30, 'BaseAngle', 60);

h = arrow([re(i+1), -im(i+1)], [re(i), -im(i)], ...
          'TipAngle', 30, 'BaseAngle', 60);

xlabel('Re');
ylabel('Im');

pbaspect([1 diff(ylim)/diff(xlim) 1]);


% Fig. 7.11: Mappings

theta = linspace(0,2*pi,200);
s1 = -.5+.9*exp(-j*theta);
g1 = (s1 - 1)./((s1 + 1).*(s1));

s3 = .5 + 1*j + .75*exp(j*theta);
g3 = (s3 - 1)./((s3 + 1).*(s3));

x = linspace(-1,1,100);
s2 = 1.5*[(x-j) (1+j*x) (-x+j) (-1-j*x)];
g2 = (s2 - 1)./((s2 + 1).*(s2));

% Fig. 7.11(a)

figure(9), clf

xl = [-2 2];
yl = [-2 2];

plot(real(s2), imag(s2), '-', ...
     real(s1), imag(s1), '--', ...
     real(s3), imag(s3), '-', ...
     [0 -1], [0 0], 'kx', ...
     [1], [0], 'ko', ...
     xl, [0 0], '-k', ...
     [0 0], yl, '-k');

i = 60;
arrow([real(s2(i)), imag(s2(i))], [real(s2(i+1)), imag(s2(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 33;
arrow([real(s1(i)), imag(s1(i))], [real(s1(i+1)), imag(s1(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 60;
arrow([real(s3(i)), imag(s3(i))], [real(s3(i+1)), imag(s3(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 130;
arrow([real(s2(i)), imag(s2(i))], [real(s2(i+1)), imag(s2(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 131;
arrow([real(s1(i)), imag(s1(i))], [real(s1(i+1)), imag(s1(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 130;
arrow([real(s3(i)), imag(s3(i))], [real(s3(i+1)), imag(s3(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

xlim(xl)
ylim(yl)

xlabel('Re');
ylabel('Im');

l = legend('$C_1$','$C_2$', '$C_3$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', 20);

pbaspect([1 diff(ylim)/diff(xlim) 1]);

% Fig. 7.11(b)

figure(10), clf

xl = [-5 2.5];
yl = [-3.5 3.5];

plot(real(g2), imag(g2), '-', ...
     real(g1), imag(g1), '--', ...
     real(g3), imag(g3), '-', ...
     xl, [0 0], '-k', ...
     [0 0], yl, '-k');

i = 360;
arrow([real(g2(i)), imag(g2(i))], [real(g2(i+1)), imag(g2(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 5;
arrow([real(g1(i)), imag(g1(i))], [real(g1(i+1)), imag(g1(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 140;
hh = arrow([real(g3(i)), imag(g3(i))], [real(g3(i+1)), imag(g3(i+1))], ...
  'TipAngle', 30, 'BaseAngle', 60);

i = 340;
arrow([real(g2(i)), imag(g2(i))], [real(g2(i+1)), imag(g2(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 93;
arrow([real(g1(i)), imag(g1(i))], [real(g1(i+1)), imag(g1(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 120;
arrow([real(g3(i)), imag(g3(i))], [real(g3(i+1)), imag(g3(i+1))], ...
      'TipAngle', 30, 'BaseAngle', 60);

xlim(xl)
ylim(yl)

xlabel('Re');
ylabel('Im');

l = legend('$G(C_1)$','$G(C_2)$', '$G(C_3)$', 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', 20);

pbaspect([1 diff(ylim)/diff(xlim) 1]);


% Nyquist #1.1

sys = tf(1,[1 1]);

rho = 10;

[re,im] = nyquist(sys, {1e-3,rho});

theta = linspace(pi/2, -pi/2, 50);
s = rho*exp(j*theta);
f = 1./(s + 1);

% Fig. 7.14

figure(11), clf

xl = [-.2 1.2];
yl = [-.6 .6];

plot(re(:), im(:), '-', ...
     re(:), -im(:), '--', ...
     real(f), imag(f), '-', ...
     xl, [0 0], '-k', ...
     [0 0], yl, '-k')

i = 40;
arrow([re(i), im(i)], [re(i+1), im(i+1)], ...
      'TipAngle', 30, 'BaseAngle', 60);

arrow([re(i+1), -im(i+1)], [re(i), -im(i)], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 25;
arrow([real(f(i)), imag(f(i))+.025], [real(f(i+1)), imag(f(i+1))+.025], ...
      'TipAngle', 30, 'BaseAngle', 60);

xlim(xl)
ylim(yl)

xlabel('Re');
ylabel('Im');

pbaspect([1 diff(ylim)/diff(xlim) 1]);


% Nyquist delay

sys = tf(1,[1 1]);

rho = 200;
tau = 1;

omega = logspace(-2,log10(rho),300);
s = j*omega;
f = (s + 1)./(s + 1 + exp(-tau*s));
re = real(f);
im = imag(f);
mag = abs(f);
phs = (180/pi)*phase(f);

% Fig. 7.15(a):

figure(12), clf

semilogx(omega, 20*log10(mag(:)'));
set(gca,'YTick',[-10 0 10]);
grid on

xlim(10.^[-1 log10(rho)]);
pbaspect([1 1/4 1]);

ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');

% Fig. 7.15(b):

figure(13), clf

semilogx(omega, phs(:));
set(gca,'YTick',[-45 0 45]);
grid on

xlim(10.^[-1 log10(rho)]);
ylim([-45 45]);

ylabel('Phase (deg)');
xlabel('\omega (rad/s)');

pbaspect([1 1/4 1]);

% Fig. 7.15(c)

figure(14), clf

xl = [0 2];
yl = [-1 1];

plot(re(:), im(:), ...
     re(:), -im(:), '--', ...
     xl, [0 0], '-k', ...
     [0 0], yl, '-k')

i = 150;
arrow([re(i), im(i)], [re(i+1), im(i+1)], ...
      'TipAngle', 30, 'BaseAngle', 60);

arrow([re(i+1), -im(i+1)], [re(i), -im(i)], ...
      'TipAngle', 30, 'BaseAngle', 60);

xlim(xl)
ylim(yl)

xlabel('Re');
ylabel('Im');

pbaspect([1 2 1]);


% Fig. 7.17

sys = tf([1 -1/2],conv([1 1],[1 1.5 1.5 1]))

[gm,pm,wgm,wpm] = margin(sys);

gm = freqresp(sys,wgm);
pm = freqresp(sys,wpm);

rho = 8;

[re,im] = nyquist(sys, {1e-3,rho});

k = 1/.4;

% Fig. 7.17(a)

figure(15), clf

xl = [-.7 1.2];
yl = [-1.2 1.2];

plot(re(:), im(:), '-', ...
     re(:), -im(:), '--', ...
     -1/k, 0, '+r', ...
     gm, 0, 'ok', ...
     [-1/k -1/k], yl, 'k-.', ...
     xl, [0 0], '-k', ...
     [0 0], yl, '-k')

text(-0.35, -1.1, '$-\alpha^{-1}$', 'FontSize', 14, 'interpreter', 'latex');

i = 40;
arrow([re(i), im(i)], [re(i+1), im(i+1)], ...
      'TipAngle', 30, 'BaseAngle', 60);

arrow([re(i+1), -im(i+1)], [re(i), -im(i)], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 70;
arrow([re(i), im(i)], [re(i+1), im(i+1)], ...
      'TipAngle', 30, 'BaseAngle', 60);

arrow([re(i+1), -im(i+1)], [re(i), -im(i)], ...
      'TipAngle', 30, 'BaseAngle', 60);

xlim(xl)
ylim(yl)

xlabel('Re');
ylabel('Im');

set(gca,'YTick',[-1 -.5 0 .5 1]);
pbaspect([1 diff(ylim)/diff(xlim) 1]);


% Fig. 7.17(b)

figure(16), clf

xl = [-0.2 1.7];
yl = [-1.2 1.2];

plot(re(:)+1/k, im(:), ...
     re(:)+1/k, -im(:), '--', ...
     0, 0, '+r', ...
     [1/k 1/k], yl, 'k-.', ...
     xl, [0 0], 'k-', ...
     [0 0], yl, '-k')

xlabel('Re');
ylabel('Im');

text(0.45, -1.1, '$\alpha^{-1}$', 'FontSize', 14, 'interpreter', 'latex');

i = 40;
arrow([re(i)+1/k, im(i)], [re(i+1)+1/k, im(i+1)], ...
      'TipAngle', 30, 'BaseAngle', 60);

arrow([re(i+1)+1/k, -im(i+1)], [re(i)+1/k, -im(i)], ...
      'TipAngle', 30, 'BaseAngle', 60);

i = 70;
arrow([re(i)+1/k, im(i)], [re(i+1)+1/k, im(i+1)], ...
      'TipAngle', 30, 'BaseAngle', 60);

arrow([re(i+1)+1/k, -im(i+1)], [re(i)+1/k, -im(i)], ...
      'TipAngle', 30, 'BaseAngle', 60);

xlim(xl)
ylim(yl)

set(gca,'YTick',[-1 -.5 0 .5 1]);
set(gca,'XTick',[0 .4 .8 1.2 1.6]);

pbaspect([1 diff(ylim)/diff(xlim) 1]);

% Fig. 7.18: Nyquist with single pole at zero

L = tf(1/4,[1 1/2 1 0])

[gm,pm,wgm,wpm] = margin(L)
gm = freqresp(L,wgm)
pm = freqresp(L,wpm)

xl = [-1 1];
yl1 = [-70 10];
yl2 = [-315 45];

[mag,phs,w] = bode(L, {10^xl(1), 10^xl(2)});
[wl,magl,wp,phsl] = basym(L);

figure(17), clf

[haxes,hline1,hline2] = ...
  plotyy(w, 20*log10([mag(:)']), ...
	 w, [phs(:)'], ...
	 'semilogx', 'semilogx');

set(haxes(1), 'XLim', 10.^xl);
set(haxes(2), 'XLim', 10.^xl);
set(haxes(1), 'YLim', yl1);
set(haxes(2), 'YLim', yl2);

set(haxes, 'Box', 'off');
set(haxes, 'YGrid', 'on');
set(haxes, 'XGrid', 'on');

set(haxes(1), 'YTick', [-60 -40 -20 0 10]);
set(haxes(2), 'YTick', [-270 -180 -90 0 45]);

set(haxes, 'PlotBoxAspectRatio', [1 1/2 1]);

ylabel(haxes(1), 'Magnitude (dB)');
ylabel(haxes(2), 'Phase (deg)');
xlabel(haxes(1), '\omega (rad/s)');

