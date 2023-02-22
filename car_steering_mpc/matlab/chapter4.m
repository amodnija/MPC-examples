% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Chapter 4

% Linearized car model
pOverBHat = 73.3
bOverMHat = 0.05
pOverMHat = pOverBHat * bOverMHat

yBar = 60
uMax = 3;

% Fig. 4.4: Step response with integral control

T = 60

% open loop
g = tf(pOverMHat, [1 bOverMHat])

% integral controller
k = tf(1, [1 0])

N = 4;
ks = [0.001 0.002 0.005 0.05]

leg = [char(ones(N, 1) * double('$K = ')) num2str(ks','%4.3f') char(ones(N, 1) * double('$~'))]

taus = zeros(N,1);
yTildes = zeros(N,1);
Hs = zeros(N,1);

t = 0 : T / 200 : T;
y = zeros(N,length(t));

% closed loop

for i = 1 : N;

  Kp = ks(i);

  % linear model
  h = feedback(Kp*g*k, 1);
  y(i,:) = step(yBar * h, t);

end

figure(1), clf

plot(t, y, [0 T], yBar*[1 1], 'k--');
xlabel('t (s)')
ylabel('y(t) (mph)')
h = legend(leg, 'Location', 'SouthEast');
set(h, 'Interpreter', 'latex')
grid on


% Fig 4.9: Response to disturbance with integral control

G = 9.8 * 3600 / 1609

thetaBar = atan(.1) % 10% slope
wBar = G/pOverMHat * sin(thetaBar)

T0 = 10
T = 200

g = tf(pOverMHat,[1 bOverMHat]);

t = 0 : T / 200 : T - T0;
y = zeros(N,length(t));

for i = 1 : N;
  Kp = ks(i);

  % linear model
  d = feedback(g, Kp*k);
  y(i,:) = step(-wBar * d, t);
end

y = [yBar*ones(N,2) yBar+y];

figure(2), clf
plot([0, T0, t+T0], y, [0 T], yBar*[1 1], 'k--');
xlabel('t (s)')
ylabel('y(t) (mph)')
ylim([35 70])
h = legend(leg, 'Location', 'SouthEast');
set(h, 'Interpreter', 'latex')
grid on

% Fig. 4.6: Step response with proportional-integral control

% Pole-zero cancelation design

% open loop
g = tf(pOverMHat, [1 bOverMHat])

% controller
k = tf([1 bOverMHat], [1 0])

N = 4;
ks = 3 ./ [90 60 30 6]
ks = round(ks*100)/100

taus = 1./(pOverMHat * ks)
tr = 2.2 * taus
kis = bOverMHat * ks

T = 15

leg = [char(ones(N, 1) * double('$K_p =')) num2str(ks','%3.2f') char(ones(N, 1) * double('$~'))]

% step response

t = 0 : T / 200 : T;
y = zeros(N,length(t));
u = zeros(N, length(t));

for i = 1 : N;
  Kp = ks(i);
  Ki = Kp * bOverMHat

  % linear model
  h = feedback(Kp*g*k, 1);
  y(i,:) = step(yBar * h, t);
end

figure(3), clf

plot(t, y, [0 T], yBar*[1 1], 'k--');
xlabel('t (s)')
ylabel('y(t) (mph)')
ylim([0 1.1*yBar])
h = legend(leg, 'Location', 'SouthEast');
set(h, 'interpreter', 'latex')
grid on


% Fig 4.10: Response to disturbance with proportional-integral control

T0 = 10
T = 200

t = 0 : T / 200 : T - T0;
y = zeros(N,length(t));

for i = 1 : N;
  Kp = ks(i);
  Ki = Kp * bOverMHat

  % linear model
  d = feedback(g, Kp*k);
  y(i,:) = step(-wBar * d, t);
end

y = [yBar*ones(N,2) yBar+y];

figure(4), clf
plot([0, T0, t+T0], y, [0 T], yBar*[1 1], 'k--');
xlabel('t (s)')
ylabel('y(t) (mph)')
ylim([35 70])
h = legend(leg, 'Location', 'SouthEast');
set(h, 'interpreter', 'latex')
grid on


% Fig. 4.15: Step response with gain mismatch

T = 20

mmatch = [1 2 1.1 0.5];
M = length(mmatch)

t = 0 : T / 200 : T;
y = zeros(M,length(t));

leg = [char(ones(M, 1) * double('$\gamma = ')) num2str(round(100*mmatch)', '%3d') char(ones(M, 1) * double('\%$~'))]

i = 3;
for l = 1 : M;

  Kp = ks(i);
  Ki = mmatch(l) * Kp * bOverMHat;
  k = tf([1 Ki/Kp], [1 0])

  % linear model
  h = feedback(Kp*g*k, 1);
  y(l,:) = step(yBar * h, t);

end

figure(5), clf

plot(t, y, [0 T], yBar*[1 1], 'k--');
xlabel('t (s)')
ylabel('y(t) (mph)')
ylim([0 1.1*yBar])
h = legend(leg, 'Location', 'SouthEast');
set(h, 'Interpreter', 'latex')
grid on

% Frequency responses

% open loop
g = tf(pOverMHat, [1 bOverMHat])

% Proportional
Kp = 0.05

kp = Kp;
Sp = feedback(1, g*kp);
Hp = feedback(g*kp, 1);
Dp = feedback(g, kp);

% Integral
Ki = 0.002

ki = Ki*tf(1,[1 0]);
Si = feedback(1, g*ki);
Hi = feedback(g*ki, 1);
Di = feedback(g, ki);

% Pole-zero cancelation design

% controller
Kp = 0.05
Ki = Kp * bOverMHat

kpi = Kp*tf([1 bOverMHat], [1 0])
Spi = feedback(1, g*kpi);
Hpi = feedback(g*kpi, 1);
Dpi = feedback(g, kpi);


% Fig. 4.7: Sensitivity with P, O and PI controllers

w0 = 2e-3
w1 = 2e0

w = logspace(log10(w0), log10(w1), 200);

N = 3;
mag = zeros(N, length(w));
phs = zeros(N, length(w));

leg = {'P', 'I','PI'}

i = 1;
[mag(i,:), phs(i,:)] = bode(Sp, w);

i = i + 1;
[mag(i,:), phs(i,:)] = bode(Si, w);

i = i + 1;
[mag(i,:), phs(i,:)] = bode(Spi, w);

figure(6), clf
semilogx(w, mag);

xlabel('\omega (rad/s)')
ylabel('|S(j\omega)|')
legend(leg, 'Location', 'NorthWest')
xlim([w0 w1])
ylim([0 2.2])
grid


% Fig. 4.11: P, O and PI controllers

leg = {'P','I','PI','open-loop'}

N = 4;
mag = zeros(N, length(w));
phs = zeros(N, length(w));

i = 1;
[mag(i,:), phs(i,:)] = bode(Dp, w);

i = i + 1;
[mag(i,:), phs(i,:)] = bode(Di, w);

i = i + 1;
[mag(i,:), phs(i,:)] = bode(Dpi, w);

i = 4;
[mag(i,:), phs(i,:)] = bode(g, w);

figure(7), clf
semilogx(w, mag);

xlabel('\omega (rad/s)')
ylabel('|D(j\omega)|')
legend(leg, 'Location', 'NorthEast')
xlim([w0 w1])
%ylim([0 2.2])
grid


% Fig. 4.11: Complementary sensitivity for P, O and PI controllers

N = 3;
mag = zeros(N, length(w));
phs = zeros(N, length(w));

leg = {'P', 'I','PI'}

i = 1;
[mag(i,:), phs(i,:)] = bode(Hp, w);

i = i + 1;
[mag(i,:), phs(i,:)] = bode(Hi, w);

i = i + 1;
[mag(i,:), phs(i,:)] = bode(Hpi, w);

figure(8), clf
semilogx(w, mag);

xlabel('\omega (rad/s)')
ylabel('|H(j\omega)|')
legend(leg, 'Location', 'NorthWest')
xlim([w0 w1])
ylim([0 2])
grid
