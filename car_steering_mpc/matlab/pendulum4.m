% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Simple Pendulum Control Part III (Simulation)

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


% select controller
ctr = tf(ctrC7)

% Simulate nonlinear pendulum with feedback

T1 = 4
thetaBar1 = pi/2 - pi/4;
theta0 = pi/2 + pi/4;

num = ctr.num{1};
den = ctr.den{1};

bs = [0 .1 .5];

N = length(bs);
y1 = cell(1,N);
t1 = cell(1,N);
i = 1;
for b = bs

  thetaBar = thetaBar1;

  [ti,x,yi] = sim('SimplePendulumWithFeedback', T1, simset('OutputVariables', 'ty', 'RelTol', 1e-5));
  t1{i} = ti;
  y1{i} = yi;

  i = i + 1;

end

leg = [ones(N, 1) * '$b = ' num2str(bs','%2.1f') ones(N, 1) * '$'];

thetaBar2 = pi/2 + pi/4;

num = ctr.num{1};
den = ctr.den{1};

bs = [0 .1 .5];

T2 = T1;

N = length(bs);
y2 = cell(1,N);
t2 = cell(1,N);
i = 1;
for b = bs

  thetaBar = thetaBar2;
  theta0 = y1{i}(end);

  [ti,x,yi] = sim('SimplePendulumWithFeedback', T2, simset('OutputVariables', 'ty', 'RelTol', 1e-5));
  t2{i} = ti;
  y2{i} = yi;

  i = i + 1;

end

Y1 = [t1; cellfun(@(x) (180/pi)*x, y1, 'UniformOutput', false)];

Y2 = [cellfun(@(x) x+T1, t2, 'UniformOutput', false); cellfun(@(x) (180/pi)*x, y2, 'UniformOutput', false)];

yl = [-45 180];

plot(Y1{:}, ...
     Y2{:}, ...
     [0 T1+T2], (180/pi)*thetaBar1*[1 1], 'k--', ...
     [0 T1+T2], (180/pi)*thetaBar2*[1 1], 'k--', ...
     [0 T1+T2], (180/pi)*0*[1 1], 'k-')

l = legend(leg, 'Location', 'SouthEast');
set(l, 'interpreter', 'latex', 'FontSize', legfnt);


ylim(yl)

xlabel('t (s)')
ylabel('\theta (deg)')
grid on

pbaspect([1 .5 1]);

set(gca, 'YTick', [-45 0 45 90 135 180]);
