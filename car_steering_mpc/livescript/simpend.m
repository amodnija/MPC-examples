%% create MPC controller object with sample time
mpc1 = mpc(SPWD_C, 0.05);
%% specify prediction horizon
mpc1.PredictionHorizon = 10;
%% specify control horizon
mpc1.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = 0;
mpc1.Model.Nominal.Y = 0;
%% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -0.3;
mpc1.MV(1).Max = 0.3;
%% specify constraints for OV
mpc1.OV(1).Min = -1;
mpc1.OV(1).Max = 1;
%% specify weights
mpc1.Weights.MV = 0;
mpc1.Weights.MVRate = 1;
mpc1.Weights.OV = 10;
mpc1.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.Model = SPWD_S;
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc1, 201, mpc1_RefSignal, mpc1_MDSignal, options);
