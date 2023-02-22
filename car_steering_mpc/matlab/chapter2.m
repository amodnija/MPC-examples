% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Chapter 2

% exponential linear response
% Fig. 2.4

ybar = 1;
T = 4;
N = 5;
bsy0s = [.3 .5; .5 1.5; 1. 0; 3. 2; -2 .99]

leg = [char(ones(N, 1) * double('$\lambda$ = ')) num2str(-bsy0s(:,1),'%+2.1f~')]

figure(1), clf

t = 0 : T / 100 : T;
y = zeros(N,length(t));
for i = 1 : N;
  b = bsy0s(i, 1);
  y0 = bsy0s(i, 2);
  y(i,:) = ybar * (1 - exp(-b * t)) + y0 * exp(-b * t);
end

plot(t, y, [0 T], ybar*[1 1], 'k--');
xlabel('t (s)')
ylabel('y(t) (mph)')
h = legend(leg, 'Location', 'NorthEast')
set(h, 'Interpreter', 'latex')
ylim([0 2])
grid

% load data for dynamic car model
load CarModel

% linear fit
N = length(td)
T = td(N)
uBar = 1

% fit ybar to last 15 points
NBar = 15
yBarHat = mean(vd(N-NBar:N))
pOverBHat = yBarHat / uBar

% fit lambda to first 11 points
Nlambda = 11
r = (yBarHat - vd) ./ yBarHat;
r = r(1 : Nlambda)
lr = log(r);
tr = td(1 : Nlambda);

lambdaHat = lr / tr
bOverMHat = - lambdaHat
pOverMHat = bOverMHat * pOverBHat

% round estimates
yBarHat = round(yBarHat*10)/10
pOverBHat = round(pOverBHat*10)/10
lambdaHat = round(lambdaHat*100)/100
bOverMHat = round(bOverMHat*100)/100
pOverMHat = round(pOverMHat*100)/100

% Fig 2.5: 
tl = 0 : 0.1 : T;
yl = yBarHat * (1 - exp(lambdaHat*tl));

figure(2), clf
plot(tl, yl, 'r', ...
  td(N-2*NBar-4:N-NBar-1), vd(N-2*NBar-4:N-NBar-1), 'kx', ...
  td(N-NBar : N), vd(N-NBar : N), 'ks', ...
  td(1:N-2*NBar-5), vd(1:N-2*NBar-5), 'ko',...
  [0 T], pOverBHat*[1 1], 'k--');
xlabel('t (s)')
ylabel('y(t) (mph)')
title(['y(t) = ' num2str(pOverBHat,3) '(1 - exp(' num2str(lambdaHat, 2) ' t))'])
ylim([0 80])
grid

% Fig 2.6: 
figure(3), clf
plot([0 tr(end)], lambdaHat * [0 tr(end)], tr, lr, 'ok');
xlabel('t (s)')
ylabel('ln r(t)')
ylim([-1.1 0])
grid

% Closed-loop step responses (linear model)

T = 30
yBar = 60
uMax = 3
kMax = uMax/yBar

N = 4;
cs = ['g';'r';'b';'m']
ks = [0 0.02 0.05 0.5]

leg = [char(ones(N, 1) * double('K = ')) num2str(ks','%3.2f') char(ones(N, 1) * double(' '))]
leg(1,:) = 'open-loop';

taus = zeros(N,1);
yTildes = zeros(N,1);
Hs = zeros(N,1);

t = 0 : T / 200 : T;
y = zeros(N,length(t));
u = zeros(N, length(t));

% open loop
uBar = yBar / pOverBHat 
yTilde = pOverBHat * uBar
y(1,:) = yTilde * (1 - exp(-bOverMHat * t));
u(1,:) = uBar * ones(size(t));

yTildes(1) = yTilde;
taus(1) = 1/bOverMHat
Hs(1) = 1

% closed loop
for i = 2 : N;
  bb = bOverMHat + pOverMHat * ks(i)
  yTilde = yBar * pOverMHat * ks(i) / (bOverMHat + pOverMHat * ks(i))
  y(i,:) = yTilde * (1 - exp(-bb * t));
  u(i,:) = ks(i) * (yBar - yTilde * (1 - exp(-bb * t)));

  yTildes(i) = yTilde;
  taus(i) = 1/bb;
  Hs(i) = yTilde / yBar;
end

% Fig. 2.8

figure(4), clf

plot(t, y, [0 T], yBar*[1 1], 'k--');

xlabel('t (s)')
ylabel('y(t) (mph)')
legend(leg, 'Location', 'SouthEast')
ylim([0 1.1*yBar])
grid on

% Fig. 2.9:

figure(5), clf

plot(t, u, [0 T], uMax*[1 1], 'k--');
xlabel('t (s)')
ylabel('u(t) (in)')
legend(leg, 'Location', 'NorthEast')
ylim([0 5])
grid on

mat = [ks' Hs 1-Hs yTildes taus log(9)*taus]

% Closed-loop step responses (nonlinear model)

alpha = 82.8
beta = 1.2

cOverMHat = alpha * bOverMHat;
dOverMHat = beta * cOverMHat;
dOverCHat = dOverMHat / cOverMHat;

% round
cOverMHat = round(cOverMHat*10)/10
dOverMHat = round(dOverMHat*10)/10
dOverCHat = round(dOverCHat*10)/10

ynl = {};
unl = {};

% nl open-loop step-response

t = linspace(0,T,200);

uBar = tan(yBar/alpha)/beta
f = @(t,v) dOverMHat*uBar - cOverMHat*tan(v/alpha);
sol = ode45(f, [0 T], 0);
ynl{1} = deval(sol, t);
unl{1} = uBar * ones(size(t));

uBar = yBar / pOverBHat 
yTilde = pOverBHat * uBar

for i = 2 : N

    % .
    % v = -(c/m)atan(y/alpha) + (d/m) sat(u)
    % u = K (ybar - v)
    
    % nl closed-loop step-response
    f = @(t,v) dOverMHat*min([3,ks(i)*(yBar-v)])-cOverMHat*tan(v/alpha);
    sol = ode45(f, [0 T], 0);
    ynl{i} = deval(sol, t);
    unl{i} = ks(i) * (yBar - ynl{i});
    unl{i}(unl{i} > 3) = 3;
    i = i + 1;

end

% Fig. 2.10

figure(6), clf

plot(t, ynl{1}, ...
     t, ynl{2}, ...
     t, ynl{3}, ...
     t, ynl{4}, ...
     [0 T], yBar*[1 1], 'k--');

xlabel('t (s)')
ylabel('y(t) (mph)')
legend(leg,'Location','SouthEast')
ylim([0 1.1*yBar]);
grid on

% Fig. 2.11

figure(7), clf

plot(t, unl{1}, ...
     t, unl{2}, ...
     t, unl{3}, ...
     t, unl{4}, ...
     [0 T], uMax*[1 1], 'k--');

xlabel('t (s)')
ylabel('u(t) (in)')
ylim([0 3.4]);
legend(leg,'Location','NorthEast')
grid on


% Closed-loop step disturbance response (linear model)

G = 9.8 * 3600 / 1609
yBar = 60
thetaBar = atan(.1) % 10% slope
wBar = G/pOverMHat * sin(thetaBar)

T0 = 10
T = 50
t = 0 : T/200 : T;

tt = [-T0 0 t];
yy = zeros(N, length(tt));

g = tf(pOverMHat,[1 bOverMHat]);
k = 1;

% open loop
el = step(-wBar * g, t);
yy(1,:) = [yBar; yBar; yBar + el];

% closed loop
for i = 2 : N;
  Kp = ks(i);
  h = feedback(Kp*k*g, 1);
  yTilde = freqresp(h, 0) * yBar;
  % linear model
  d = feedback(g, Kp*k);
  [el, tl] = step(-wBar * d, t);
  yy(i,:) = [yTilde; yTilde; yTilde + el];
end


figure(8)

plot(tt, yy, [-T0 T], yBar*[1 1], 'k--', [0 0], 2*yBar*[0 1], 'k--')

xlabel('t (s)')
ylabel('y(t) (mph)')
legend(leg,'Location','SouthWest')
ylim([15 yBar+5])
grid on


% Closed-loop ramp response (linear model)

mu = 12

T = 15

leg(1,:) = 'reference';

t = 0 : T / 100 : T;

yy = zeros(2*(N-1), length(t));

for i = 2 : N

  k = ks(i)

  b1 = k * pOverMHat;
  a1 = bOverMHat + k * pOverMHat;

  % linearized step-response
  yBar = mu*b1/a1^2;
  yy(i-1,:) = yBar * (exp(-a1*t) - 1 + a1*t);
  yy(N+i-2,:) = yBar * (-1 + a1*t);

end

figure(9)

plot([0 T], [0 mu*T], 'k--', t, yy, '-')

xlabel('t (s)')
ylabel('y(t) (mph)')
legend(leg,'Location','NorthWest')
ylim([0 150]);
grid on
