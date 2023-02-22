% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Chapter 1

% load data for static car model
load CarModel

% Fig. 1.4: pedal excursion (us) x velocity (vs)
figure(1), clf
plot(us, vs, 'x')
xlabel('pedal excursion (in)')
ylabel('velocity (mph)')
ylim([0 120])
grid

% least-square fit
f = fit(us', vs', 'a*atan(b*x)', 'StartPoint', [80,1])
% round to one digit
alpha = round(f.a*10)/10
beta = round(f.b*10)/10

% plot fitted data
u = 0 : 0.01 : 3;
v = alpha*atan(beta*u);

% Fig. 1.5: pedal excursion (us) x velocity (vs) with nonlinear fit
figure(2), clf
plot(u, v, '-', us, vs, 'x');
xlabel('pedal excursion (in)')
ylabel('velocity (mph)')
title(['y = ' num2str(alpha,3) ' atan(' num2str(beta,2) ' u)'])
ylim([0 120])
grid

% linear fit
g1 = vs / us;
g1 = round(g1*10)/10

% linearized nonlinear model
g2 = round(alpha*beta*10)/10

% Fig. 1.6: pedal excursion (us) x velocity (vs) with nonlinear and
% linear fits
figure(3), clf
plot(u, v, '-', u, g1*u, '--', u, g2*u, '--', us, vs, 'x');
xlabel('pedal excursion (in)')
ylabel('velocity (mph)')
ylim([0 120])
title(['y = ' num2str(alpha,3) ' atan(' num2str(beta,2) ' u),  ' ...
       'y = ' num2str(g1,3) ' u,  ' ...
       'y = ' num2str(g2,4) ' u ' ...
      ])
grid

%
% Static model for the car velocity control
% Static feedback control
%

% pick extra point in the middle
g3 = 73.3

% build table of closed loop gains
ks = [0.02 0.05 0.5 1 3]
ks = [0.02 0.05 1 3]
mat = [
g1 * ks ./ ( 1 + g1 * ks);
g3 * ks ./ ( 1 + g3 * ks);
g2 * ks ./ ( 1 + g2 * ks);
];

% Table 1.1: Linear closed-loop gains
printmat(mat, 'Table 1.1', num2str([g1,g3,g2]), num2str(ks))

% closed loop nonlinear gains
u = 0 : 0.01 : 3;
ybars = 40 : 5 : 100;
i = 1;
H = [];
for k = ks
  l = 1;
  for ybar = ybars
    f = inline(['y - ' num2str(alpha) '*atan(' num2str(beta) '*' num2str(k) '*(' num2str(ybar) '-y))'], 'y');
    H(i,l) = fsolve(f, ybar, optimoptions('fsolve','Display','off')) / ybar;
    l = l + 1;
  end
  i = i + 1;
end

leg = [char(ones(4, 1) * double('K = ')) num2str(ks','%3.2f') char(ones(4, 1)) * ' '];

% Fig. 1.9: nonlinear closed-loop gains
figure(4), clf
plot(ybars, H, [40 100], [1 1], 'k--')
xlabel('velocity (mph)')
ylabel('velocity/target')
legend(leg, 'Location', 'SouthWest');
ylim([0.5 1.02])
grid
