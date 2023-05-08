function [t0, x0, u0] = shift(T, t0, x0, u,f)
st = x0; 
con = u(1,:)';
f_value = f(st,con); % use the function we have created in the MPC setup to generate f_value, which is a factor by which we can multiply the sampling time to obtain the delta between two states
st = st+ (T*f_value); % predict next state by multiplying T*f_value and adding it to the current state.
x0 = full(st);

t0 = t0 + T;
u0 = [u(2:size(u,1),:);u(size(u,1),:)]; % initialize u0 with the current optimal solution
end