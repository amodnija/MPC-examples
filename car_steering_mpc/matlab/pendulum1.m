% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Simple Pendulum Control Part I

legfnt = 13;

% Pendulum parameters

m = 0.5
l = 0.3
r = l/2
b = 0
g = 9.8
J = m*l^2/12
Jr = (J+m*r^2)

% models linearized around equilibria
Gpi = tf(1/Jr, [1, b/Jr, -m*r*g/Jr]);
G0 = tf(1/Jr, [1, b/Jr, m*r*g/Jr]);

den = Gpi.den{1};
wn = sqrt(-den(3)/den(1))
zeta = den(2)/(2*wn*den(1))


% Fig. 6.14: PD control of the simple pendulum

% sqrt(2) * natural frequency, zeta = 0.5
Kp = 3*m*r*g
Kd = 2*sqrt(m*r*g*Jr)-b
z = Kp/Kd

ctrPD = Kp + Kd * tf([1 0],1)

H = feedback(ctrPD*G0,1)

den = H.den{1};
wn = sqrt(den(3)/den(1))
zeta0 = den(2)/(2*wn*den(1))

H = feedback(ctrPD*Gpi,1)

den = H.den{1};
wn = sqrt(den(3)/den(1))
zetapi = den(2)/(2*wn*den(1))

wd = wn*sqrt(1 - zeta^2)
fd = wd/(2*pi)
Td = 1/fd

L = ctrPD*Gpi;

t = 0 : 0.1 : 2*pi;
circ = wn*cos(t) + j*wn*sin(t);

pz = {};
pz{1} = pole(L);
pz{2} = zero(L);
pz{3} = pole(H);

rl = rlocus(L);

a = (sum(pz{1}) - sum(pz{2}))/(3-1)

figure(1), clf

xl = [-18,8]
yl = [-11,11]

plot(real(pz{3}), imag(pz{3}), 'd', ...
     real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(rl'), imag(rl'), ...
     real(circ), imag(circ), 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zetapi))], 'k:', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(yl)/diff(xl) 1]);


% Fig. 6.15: Add integral control pole

ctrPDI = ctrPD * tf(1,[1 0])

H = feedback(ctrPDI*Gpi,1)

L = ctrPDI*Gpi;

pz = {};
pz{1} = pole(L);
pz{2} = zero(L);
pz{3} = pole(H);

rl = rlocus(L);

a = (sum(pz{1}) - sum(pz{2}))/(3-1)

figure(2), clf

xl = [-12,8];
yl = [-30,30];

plot(real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(rl'), imag(rl'), ...
     a*[1 1], yl, 'k-.', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(xl)/diff(yl) 1]);


% Fig. 6.16 (a): Place pole 8 times farther than zero

pd = 8*zero(ctrPD)
ctrPDP = 0.9*ctrPD * tf(-pd,[1 -pd])
ctrPDP8 = ctrPDP;

H = feedback(ctrPDP*Gpi,1)

p = pole(H);
p(imag(p) == 0) = []
pp = conv([1 -p(1)],[1 -p(2)])
zetapi = pp(2)/(2*sqrt(pp(3)))

L = ctrPDP*Gpi;

pz = {};
pz{1} = pole(L);
pz{2} = zero(L);
pz{3} = pole(H);

rl = rlocus(L);

a = (sum(pz{1}) - sum(pz{2}))/(3-1)

t = 0 : 0.1 : 2*pi;
circ = wn*cos(t) + j*wn*sin(t);

figure(3), clf

xl = [-70,10];
yl = [-13,13];

plot(real(pz{3}), imag(pz{3}), 'd', ...
     real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(rl'), imag(rl'), ...
     a*[1 1], yl, 'k-.', ...
     real(circ), imag(circ), 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zetapi))], 'k:', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(yl)/diff(xl) 1]);


% Fig. 6.16 (b): Place pole 2 times farther than zero

pd = 2*zero(ctrPD)
ctrPDP = 0.8*ctrPD * tf(-pd,[1 -pd])
ctrPDP2 = ctrPDP;

H = feedback(ctrPDP*Gpi,1)

p = pole(H);
p(imag(p) == 0) = []
pp = conv([1 -p(1)],[1 -p(2)])
zetapi = pp(2)/(2*sqrt(pp(3)))

L = ctrPDP*Gpi;

pz = {};
pz{1} = pole(L);
pz{2} = zero(L);
pz{3} = pole(H);

rl = rlocus(L);

a = (sum(pz{1}) - sum(pz{2}))/(3-1)

t = 0 : 0.1 : 2*pi;
circ = wn*cos(t) + j*wn*sin(t);

figure(4), clf

xl = [-25,10];
yl = [-11,11];

plot(real(pz{3}), imag(pz{3}), 'd', ...
     real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(rl'), imag(rl'), ...
     a*[1 1], yl, 'k-.', ...
     real(circ), imag(circ), 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zetapi))], 'k:', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(yl)/diff(xl) 1]);


% Fig 6.18: Place pole 2 times and adjust zero

pd = 2*zero(ctrPD)

ctrPDD = 0.9*Kp*tf([1.4*Kd/Kp 1],1)/1.45

ctrPDP = ctrPDD * tf(-pd,[1 -pd])
ctrPDP2z = ctrPDP;

zpk(ctrPDP2z)

H = feedback(ctrPDP*Gpi,1)

p = pole(H);
p(imag(p) == 0) = []
pp = conv([1 -p(1)],[1 -p(2)])
zetapi = pp(2)/(2*sqrt(pp(3)))

L = ctrPDP*Gpi;

pz = {};
pz{1} = pole(L);
pz{2} = zero(L);
pz{3} = pole(H);

rl = rlocus(L);

a = (sum(pz{1}) - sum(pz{2}))/(3-1)

t = 0 : 0.1 : 2*pi;
circ = wn*cos(t) + j*wn*sin(t);

figure(5), clf

xl = [-25,10];
yl = [-11,11];

plot(real(pz{3}), imag(pz{3}), 'd', ...
     real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(rl'), imag(rl'), ...
     a*[1 1], yl, 'k-.', ...
     real(circ), imag(circ), 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zetapi))], 'k:', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(yl)/diff(xl) 1]);

% Fig. 6.19: root locus for stable equilibrium

H0 = feedback(ctrPDP2z*G0,1)

pp = pole(H0);
pp(imag(pp) == 0) = [];
pp = conv([1 -pp(1)],[1 -pp(2)]);
zeta00 = pp(2)/(2*sqrt(pp(3)));
wn00 = sqrt(pp(3));

L0 = ctrPDP2z*G0;

pz0 = {};
pz0{1} = pole(L0);
pz0{2} = zero(L0);
pz0{3} = pole(H0);

rl0 = rlocus(L0);

a0 = (sum(pz{1}) - sum(pz{2}))/(3-1)

figure(6), clf

xl = [-20,10];
yl = [-16,16];

plot(real(pz{3}), imag(pz{3}), 's', ...
     real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(pz0{3}), imag(pz0{3}), 'd', ...
     real(pz0{2}), imag(pz0{2}), 'o', ...
     real(pz0{1}), imag(pz0{1}), 'x', ...
     real(rl'), imag(rl'), '--', ...
     real(rl0'), imag(rl0'), '-', ...
     a0*[1 1], yl, 'k-.', ...
     real(circ), imag(circ), 'k:', ...
     (wn00/wn)*real(circ), (wn00/wn)*imag(circ), 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zetapi))], 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zeta00))], 'k:', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(yl)/diff(xl) 1]);


% Fig: 6.21: Place pole 2 times adjust zero + integrator

Gcl = feedback(Gpi,ctrPDP2z)

ctrI = 1.197*tf(1,[1 0])
ctrIalt = 2.8*tf(1,[1 0])

H = feedback(ctrI*Gcl,1)

pp = pole(H);
pp(imag(pp) == 0) = []
pp = conv([1 -pp(1)],[1 -pp(2)])
wn = sqrt(pp(3)/pp(1))
zetapi = pp(2)/(2*sqrt(pp(3)))

Halt = feedback(ctrIalt*Gcl,1)

pp = pole(Halt);
pp(imag(pp) == 0) = []
pp = conv([1 -pp(1)],[1 -pp(2)])
wnAlt = sqrt(pp(3)/pp(1))
zetaAlt = pp(2)/(2*sqrt(pp(3)))

L = ctrI*Gcl;

pz = {};
pz{1} = pole(L);
pz{2} = zero(L);
pz{3} = pole(H);
pz{4} = pole(Halt);

rl = rlocus(L);

a = (sum(pz{1}) - sum(pz{2}))/(3-1)

t = 0 : 0.1 : 2*pi;
circ = wn*cos(t) + j*wn*sin(t);

figure(7), clf

xl = [-22,8];
yl = [-10,10];

plot(real(pz{3}), imag(pz{3}), 'd', ...
     real(pz{4}), imag(pz{4}), 's', ...
     real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(rl'), imag(rl'), '-', ...
     [a a+10], [0 10*tan(pi/3)], 'k-.', ...
     [a a+10], [0 -10*tan(pi/3)], 'k-.', ...
     real(circ), imag(circ), 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zetapi))], 'k:', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(yl)/diff(xl) 1]);


% Fig: 6.23: Complete controller root-locus

ctr = ctrI + ctrPDP2z
zpk(ctr)

% pause

H = feedback(ctr*Gpi,1)

pp = pole(H)

den = conv([1 -(pp(3)+pp(4))/2], [1 -(pp(3)+pp(4))/2]);
wn = sqrt(den(3)/den(1))
zeta = den(2)/(2*wn*den(1))
wd = wn*sqrt(1-zeta^2)

tp = pi/(wd)
PO = 100*exp(-zeta*pi/sqrt(1-zeta^2))
ts = 3.9/(zeta*wn)
tr = 2*pi*(.16+.14*zeta+.24*zeta^3)/(wn)
tau = 2*pi*(.19+.1*zeta+.0054*zeta^3)/(wn)

den = conv([1 -pp(1)], [1 -pp(2)]);
wn = sqrt(den(3)/den(1))
zeta = den(2)/(2*wn*den(1))
wd = wn*sqrt(1-zeta^2)

tp = pi/(wd)
PO = 100*exp(-zeta*pi/sqrt(1-zeta^2))
ts = 3.9/(zeta*wn)
tr = 2*pi*(.16+.14*zeta+.24*zeta^3)/(wn)
tau = 2*pi*(.19+.1*zeta+.0054*zeta^3)/(wn)


% Direct root-locus design

ctrRL = ctr;

H = feedback(ctrRL*Gpi,1)

pp = pole(H);
pp(imag(pp) == 0) = []
pp = conv([1 -pp(1)],[1 -pp(2)])
zeta = pp(2)/(2*sqrt(pp(3)))

L = ctrRL*Gpi;

pz = {};
pz{1} = pole(L);
pz{2} = zero(L);
pz{3} = pole(H);

rl = rlocus(L);

a = (sum(pz{1}) - sum(pz{2}))/(length(pz{1})-length(pz{2}))

t = 0 : 0.1 : 2*pi;
circ = wn*cos(t) + j*wn*sin(t);

figure(8), clf

xl = [-20,10];
yl = [-10,10];

plot(real(pz{3}), imag(pz{3}), 'd', ...
     real(pz{2}), imag(pz{2}), 'o', ...
     real(pz{1}), imag(pz{1}), 'x', ...
     real(rl'), imag(rl'), '-', ...
     a*[1 1], yl, 'k-.', ...
     real(circ), imag(circ), 'k:', ...
     [0 yl(1)], [0 yl(2)/tan(asin(zetapi))], 'k:', ...
     xl, [0 0], 'k--', ...
     [0 0], yl, 'k--')

xlabel('Re')
ylabel('Im')
xlim(xl)
ylim(yl)
set(gca, 'PlotBoxAspectRatio', [1 diff(yl)/diff(xl) 1]);


% Fig. 6.17: Frequency response of controllers

w = logspace(0,2.3,100);

[magPD,phasePD] = bode(ctrPD, w);
[magPDP8,phasePDP8] = bode(ctrPDP8, w);
[magPDP2,phasePDP2] = bode(ctrPDP2, w);
[magPDP2z,phasePDP2z] = bode(ctrPDP2z, w);

[wlPD,maglPD,wpPD,phslPD] = basym(ctrPD);
[wlPDP8,maglPDP8,wpPDP8,phslPDP8] = basym(ctrPDP8);
[wlPDP2,maglPDP2,wpPDP2,phslPDP2] = basym(ctrPDP2);
[wlPDP2z,maglPDP2z,wpPDP2z,phslPDP2z] = basym(ctrPDP2z);

maglPD(2) = 20*log10(abs(freqresp(ctrPD,wlPD(2))));
maglPDP8(3) = 20*log10(abs(freqresp(ctrPDP8,wlPDP8(3))));
maglPDP2z(2) = 20*log10(abs(freqresp(ctrPDP2z,wlPDP2z(2))));
maglPDP2z(3) = 20*log10(abs(freqresp(ctrPDP2z,wlPDP2z(3))));

figure(9), clf

semilogx(w, 20*log10([magPD(:)'; magPDP8(:)'; magPDP2(:)'; magPDP2z(:)']), ...
         [wlPD(2) wlPD(2)], [0 maglPD(2)], 'k--', ...
         [wlPDP8(3) wlPDP8(3)], [0 maglPDP8(3)], 'k--', ...
         [wlPDP2z(2) wlPDP2z(2)], [0 maglPDP2z(2)], 'k--', ...
         [wlPDP2z(3) wlPDP2z(3)], [0 maglPDP2z(3)], 'k--');
xlim(10.^[0 2.3]);
pbaspect([1 1/2 1]);

text(1.1*wlPD(2), 2.5, '$z$','interpreter','latex','FontSize',15,'HorizontalAlignment','left');

text(1.1*wlPDP8(3), 2.5, '$8 z$','interpreter','latex','FontSize',15,'HorizontalAlignment','left');

text(1.1*wlPDP2z(2), 2.5, '$\tilde{z}$','interpreter','latex','FontSize',15,'HorizontalAlignment','left');

text(1.1*wlPDP2z(3), 2.5, '$2 z$','interpreter','latex','FontSize',15,'HorizontalAlignment','left');

h = legend('$(s + z)$', '$(s + z)/(s + 8 z)$~~~~', '$(s + z)/(s + 2 z)$~~~~', '$(s + \tilde{z})/(s + 2 z)$~~~~', 'Location', 'NorthWest');
set(h, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');


% Fig. 6.17: Frequency response of lead controller

w = logspace(-.8,2.3,100);
[magPDP2z,phasePDP2z] = bode(ctrPDP2z, w);
[magCtr,phaseCtr] = bode(ctr, w);

[wlPDP2z,maglPDP2z,wpPDP2z,phslPDP2z] = basym(ctrPDP2z);
[wlCtr,maglCtr,wpCtr,phsCtr] = basym(ctr);

figure(10),clf

semilogx(w, 20*log10([magCtr(:)'; magPDP2z(:)']));
xlim(10.^[-.8 2.3]);
pbaspect([1 1/2 1]);

l = legend('$C_6$~', '$C_\mathrm{lead}$', 'Location','SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);
ylabel('Magnitude (dB)');
xlabel('\omega (rad/s)');
