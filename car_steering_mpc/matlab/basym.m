function [wm, mag, wp, phase] = basym(L)
% Compute asymptotes in Bode plots
%

% SISO calculation
% TODO: check for SISO
[z,p,k] = zpkdata(L);
z = z{:};
p = p{:};

m = length(z) - 1;
n = length(p) - 1;

%  poles and zeros at the origin
indZ = find(z == 0);
indP = find(p == 0);
slope = 20*(length(indZ) - length(indP));
pslope = 0;
phase = 90*(length(indZ) - length(indP));

% remove poles and zeros at the origin
z(indZ) = [];
p(indP) = [];

% sort poles and zeros
w = [z; p];
t = char(['z'*ones(length(z),1); 'p'*ones(length(p),1)]);
s = sign(w);
w = abs(w);

[ws,ind] = sort(w);
ts = t(ind);
ss = s(ind);

% approximate magnitude

% origin is one decade below first pole/zero
wm = ws(1)/10;

if ( slope ~= 0 )
  mag = slope*log10(wm(1));
else
  mag = 0;
end

for i = 1 : length(ws)

  wm(i+1) = ws(i);
  mag(i+1) = mag(i) + slope*(log10(wm(i+1)/wm(i)));

  if ( ts(i) == 'p' )
    slope = slope - 20;
  elseif ( ts(i) == 'z' )
    slope = slope + 20;
  end
    
end

% add end point
i = i + 1;
wm(i+1) = 10*ws(end);
mag(i+1) = mag(i) + slope*(log10(wm(i+1)/wm(i)));

% calculate DC gain
dcGain = k * prod(-z) / prod(-p);
infGainSwitch = sign(dcGain * k);
mag = mag + 20*log10(abs(dcGain));

% approximate phase
wp = [ws/10 10*ws];
wp = unique(wp(:));
slope = zeros(size(wp));

% compute slope
for i = 1 : length(ws)

  ind = find(wp >= ws(i)/10 & wp < 10*ws(i));

  if ( (ts(i) == 'p' && ss(i) < 0) || (ts(i) == 'z' && ss(i) > 0) )
    slope(ind) = slope(ind) - 45;
  elseif ( (ts(i) == 'z' && ss(i) < 0) || (ts(i) == 'p' && ss(i) > 0) )
    slope(ind) = slope(ind) + 45;
  end
   
end

% compute phase

for i = 1 : length(wp)-1
  phase(i+1) = phase(i) + slope(i) * log10(wp(i+1)/wp(i));
end

%wp
%slope
%phase

% add DC phase
endPhase = infGainSwitch*90*(m-n);
if dcGain < 0
  phase = phase - 180;
end

% add pole/zero offset
%if phase(end) ~= (endPhase)
%  k = endPhase - phase(end);
%  if abs(mod(k,360)) > 5
%    warning('phase mismatch');
%  end
%  phase = phase + k;
%end

% add end points
phase = [phase(1); phase'; phase(end)];
wp = [wp(1)/10; wp; 10*wp(end)];
