% Fundamentals of Linear Control
% Mauricio de Oliveira
% Supplemental material for Chapter 6

legfnt = 13;

% Linearized car model
pOverBHat = 73.3
bOverMHat = 0.05
pOverMHat = pOverBHat * bOverMHat

% Fig. 6.9: Root-locus for proportional control

G = tf(pOverMHat, [1 bOverMHat]);
L = G;

N = 4;
ks = [0 0.02 0.05 0.5];
mrkrs = {'x', 'd', 's', '^'};

r = rlocus(L);
p = {};

i = 1
p{i} = pole(L);
for i = 2 : N;
  p{i} = pole(feedback(ks(i)*L,1))
end

leg = [char(ones(N, 1) * double('$\alpha = ')) num2str(ks', '%4.3f') char(ones(N, 1) * double('$~~'))]
leg(1,1:9) = 'open-loop';
leg(1,10:end) = ' ';

figure(1), clf

for i = 1 : N
  plot(real(p{i}), imag(p{i}), mrkrs{i}), hold on
end
xlim([-2, .1])
ylim([-.1, .5])

plot(real(r'), imag(r'), ...
     xlim, [0 0], 'k--', ...
     [0 0], ylim, 'k--')
hold off

h = legend(leg,'Location','NorthWest');
set(h, 'interpreter', 'latex', 'FontSize', legfnt);
xlabel('Re')
ylabel('Im')



% Fig. 6.11: Root-locus for integral control

K = tf(1, [1 0]);
L = K*G;

r = rlocus(L);

N = 4;
ks = [0 0.002 0.005 0.05];

r = rlocus(L);
p = {};

i = 1
p{i} = pole(L);
for i = 2 : N;
  p{i} = pole(feedback(ks(i)*L,1))
end

leg = [char(ones(N, 1) * double('$\alpha = ')) num2str(ks', '%4.3f') char(ones(N, 1) * double('$~~'))]
leg(1,1:9) = 'open-loop';
leg(1,10:end) = ' ';

figure(2), clf

for i = 1 : N
  plot(real(p{i}), imag(p{i}), mrkrs{i}), hold on
end
xlim([-.07, .02])
ylim(.5*[-1, 1])

plot(real(r'), imag(r'), ...
     xlim, [0 0], 'k--', ...
     [0 0], ylim, 'k--')
hold off

h = legend(leg,'Location','NorthWest');
set(h, 'interpreter', 'latex', 'FontSize', legfnt);
xlabel('Re')
ylabel('Im')


% Root locus PI control with pole zero cancelation

mmatch = [2 1.1 1 0.5];
M = length(mmatch);

r = {};
pz = {};;
alpha = 0.1;

for i = 1 : M

  Kp = 0.1;
  Ki = mmatch(i) * Kp * bOverMHat;
  K = tf([1 Ki/Kp], [1 0]);
  L = K*G;
 
  r{i} = rlocus(L);
  pz{i,1} = pole(L);
  pz{i,2} = zero(L);
  pz{i,3} = pole(feedback(alpha*L,1));

end

figure(3), clf

leg = ['$\alpha = ' num2str(alpha, '%3.2f') '$~~'];

for i = 1 : M

  subplot(M,1,i)

  xlim(2.5*[-.16, .02])
  ylim(.1*[-1, 1])

  plot(real(pz{i,3}), imag(pz{i,3}), 'd', ...
       real(pz{i,1}), imag(pz{i,1}), 'x', ...
       real(pz{i,2}), imag(pz{i,2}), 'o', ...
       real(r{i}'), imag(r{i}'), ...
       xlim, [0 0], 'k--', ...
       [0 0], ylim, 'k--')
  xlim([-.4, .05])
  ylim([-.1, .1])

  if (i == M)
    xlabel('Re')
    hleg = legend(leg, 'Location', 'SouthWest');
    set(hleg, 'interpreter', 'latex', 'FontSize', legfnt, ...
        'Position', [0.09 0.03 0.13 0.00])
  end
  ylabel('Im')

end

