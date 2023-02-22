% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Chapter 5

% Linearized car model
pOverBHat = 73.3
bOverMHat = 0.05
pOverMHat = pOverBHat * bOverMHat

alpha = 82.8
beta = 1.2

cOverMHat = alpha * bOverMHat;
dOverMHat = beta * cOverMHat;
dOverCHat = dOverMHat / cOverMHat;

yBar = 60
uMax = 3;

% Closed-loop step responses (nonlinear model)

% .
% v = -(c/m)atan(v/alpha) + (d/m) sat(u)
% .
% ei = e
% e = (ybar - v)
% u = Kp e + Ki ei

% .
% x = [-(c/m)atan(x(1)/alpha) + (d/m) sat(Kp (ybar - x(1)) + Ki x(2))]
%     [ ybar - x(1) ]

xnl = {};
ynl = {};
einl = {};
unl = {};

ks = [0.03 0.05 0.1 0.5]
N = length(ks)

leg = [char(ones(N, 1) * double('$K_p = ')) num2str(ks','%3.2f') char(ones(N, 1) * double('$~'))]

T = 15;
t = linspace(0, T, 200);

for i = 1 : N

    % nl closed-loop step-response
    Kp = ks(i);
    Ki = bOverMHat*Kp;
    f = @(t,x) [dOverMHat*min([3,Kp*(yBar-x(1))+Ki*x(2)])- ...
                cOverMHat*tan(x(1)/alpha); yBar-x(1)];
    sol = ode45(f, [0 T], [0; 0]);
    xnl{i} = deval(sol, t);
    ynl{i} = xnl{i}(1,:);
    xcnl{i} = Ki*xnl{i}(2,:);
    enl{i} = yBar-ynl{i};
    unl{i} = Kp*(yBar-ynl{i})+Ki*xcnl{i};
    unl{i}(unl{i} > 3) = 3;
    i = i + 1;
    
end

% Fig. 2.10

figure(1), clf

plot(t, ynl{1}, ...
     t, ynl{2}, ...
     t, ynl{3}, ...
     t, ynl{4}, ...
     [0 T], yBar*[1 1], 'k--');

xlabel('t')
ylabel('y(t)')
h = legend(leg,'Location','SouthEast');
set(h, 'interpreter', 'latex')
ylim([0 1.1*yBar]);
grid on

% Fig. 2.11

figure(2), clf

plot(t, unl{1}, ...
     t, unl{2}, ...
     t, unl{3}, ...
     t, unl{4}, ...
     [0 T], uMax*[1 1], 'k--');

xlabel('t')
ylabel('u(t)')
ylim([-2.5 3.4]);
h = legend(leg,'Location','NorthEast');
set(h, 'interpreter', 'latex')
grid on


T = 80;
t = linspace(0, T, 200);

for i = 1 : N

    % nl closed-loop step-response
    Kp = ks(i);
    Ki = bOverMHat*Kp;
    f = @(t,x) [dOverMHat*min([3,Kp*(yBar-x(1))+Ki*x(2)])- ...
                cOverMHat*tan(x(1)/alpha); yBar-x(1)];
    sol = ode45(f, [0 T], [0; 0]);
    xnl{i} = deval(sol, t);
    ynl{i} = xnl{i}(1,:);
    xcnl{i} = Ki*xnl{i}(2,:);
    enl{i} = yBar-ynl{i};
    unl{i} = Kp*(yBar-ynl{i})+Ki*xcnl{i};
    unl{i}(unl{i} > 3) = 3;
    i = i + 1;
    
end

% Fig. 2.11

figure(3), clf

plot(t, xcnl{1}, ...
     t, xcnl{2}, ...
     t, xcnl{3}, ...
     t, xcnl{4});

xlabel('t')
ylabel('K_i x_c(t)')
h = legend(leg,'Location','NorthEast');
set(h, 'interpreter', 'latex')
grid on
