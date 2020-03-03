%% Simulate full-state feedback controlled doulble inverted pendulum

% Author: Maarten Afschrift


clear all;

%% path information
addpath(genpath(pwd));

%% Pendulum properties

% model properties
P.m = 70; % total mass of the subject [kg]
P.l = 1.7; % total height of the subject [m]

% base of support: In this case 5 cm distance between heel and ankle, and 20 cm between ankle and toes
P.BOS = [-0.05 0.2];

% initial posture 
P.qinit = [-3; 0]*pi./180; % initial state is forward lean of 3 deg

%% inital guess based on LQR solution
% We compute the LQR gains to have a good initial guess. This is only
% background information

K_LQR = Get_LQRGains(P.m,P.l);

%% Formulate optimal control problem

% The double pendulum is driven by full state feedback:
%   T = K x 
% with the state x = [q qd]'

% The goal is to optimize the matrix K to contract a perturbation with
% minimal effort (joint moments squared) and minmial com movement

% we use casadi to formulate the OCP and solve it with ipopt. The OCP is
% fomulated with a collocation approach with an euler integration scheme

% import casadi
import casadi.*;

% OCP settings
S.dt    = 0.01;    % time step
S.tsim  = 8;
S.tvect = 0:S.dt:S.tsim;
S.N     = length(S.tvect);

% create opti variables
opti = casadi.Opti();

Ku = opti.variable(2,4);        % feedback gains
K = Ku * 200;                   % scale feedback gains (needed for optimization)
q = opti.variable(2,S.N+1);     % angles
qd = opti.variable(2,S.N+1);    % velocities
qdd = opti.variable(2,S.N);     % accelerations

% bounds
opti.subject_to(-2*pi < q < 2*pi);
opti.subject_to(-10*pi < qd < 10*pi);
opti.subject_to(-20*pi < qdd < 20*pi);

% perturbation vector
t0      = 0;    % start perturbation
tend    = 1;    % end perturbation
tp      = t0:S.dt:tend;
ap      = zeros(1,S.N);
ap(1,1:length(tp)) = sin(tp*(tend*2*pi));   % platform acceleration is a sine wave

% constraint: integration scheme
x = [q(:,1:S.N);   qd(:,1:S.N)];        % x(t)
z = [q(:,2:S.N+1); qd(:,2:S.N+1)];      % x(t+1)
u = [qd(:,1:S.N);  qdd];                % xdot(t)
c_Integration =  eulerIntegrator(x,z,u,S.dt);   % (x(t+1) - x(t)) - xdot(t)*dt = 0 ;

% constraint feedback torques
Tid     = f_Tid(ap,P.l,P.m,q(1,1:S.N),q(2,1:S.N),qd(1,1:S.N),qd(2,1:S.N),qdd(1,1:S.N),qdd(2,1:S.N));    % solve inverse dynamics
c_FB    = Tid - K * x; % full state feedback (T = Kx)

% COM location
COM = f_COMx(P.l,P.m,q(1,1:S.N),q(2,1:S.N));

% COP location
COP = f_COP(ap,P.l,P.m,q(1,1:S.N),q(2,1:S.N),qd(1,1:S.N),qd(2,1:S.N),qdd(1,1:S.N),qdd(2,1:S.N));

% impose constraints using opti
opti.subject_to(c_Integration == 0);
opti.subject_to(c_FB == 0);

% constraints related to initial and final postures
opti.subject_to(q(:,1) == P.qinit);
opti.subject_to(qd(:,1) == 0);

% upright posture after 5 seconds
it5 = find(S.tvect>5,1);
opti.subject_to(-0.03 <  q(:,it5) < 0.03);
opti.subject_to(-0.01 <  qd(:,it5) < 0.01);
opti.subject_to(-0.1 <  qdd(:,it5) < 0.1);

% objective function
JT = sumsqr(Tid);
JCOM = sumsqr(COM);
JLQR = sumsqr(K-K_LQR);
J = JT*0.001 + JCOM + 0.00001*JLQR;

% settings for ipopt
% optionssol.ipopt.hessian_approximation  = 'limited-memory';
optionssol.ipopt.nlp_scaling_method     = 'gradient-based';
optionssol.ipopt.linear_solver          = 'mumps';
optionssol.ipopt.tol                    = 1e-6;
optionssol.ipopt.acceptable_tol         = 1e-5;

% initial guess based on LQR solution (forward simulation with LQR gains)
optic = opti.copy();
optic.subject_to(K == K_LQR);
optic.minimize(0);
optic.solver('ipopt',optionssol);
SolGuess = optic.solve();

opti.set_initial(K, K_LQR);
opti.set_initial(q,value(SolGuess,q));
opti.set_initial(qd,value(SolGuess,qd));
opti.set_initial(qdd,value(SolGuess,qdd));

% constrain COP position in BOS
opti.subject_to(P.BOS(1)< COP < P.BOS(2));

% % solve the OCP problem
opti.solver('ipopt',optionssol);
opti.minimize(J);   % minimimze objective function
sol = opti.solve();

%% Store the solution

% store the results
R.q     = value(sol,q)';    % joint angles
R.qd    = value(sol,qd)';   % anglular velocities
R.qdd   = value(sol,qdd)';  % angular accelerations
R.T     = value(sol,Tid)';  % joint moments
R.COP   = value(sol,COP)';  % location COP
R.COM   = value(sol,COM)';  % location COM
R.t     = S.tvect;          % time vector
R.ap    = ap';              % platform acceleration
R.S     = S;                % settings structure
R.P     = P;                % model properties
R.K     = value(sol,K);     % feedback gains
R.J     = value(sol,J);     % value objective function

%% Plot results

figure();
subplot(2,2,1)
plot(R.t,R.q(1:end-1,:)*180/pi);
xlabel('Time [s]');
ylabel('joint angles [deg]');

subplot(2,2,2)
plot(R.t,R.T);
xlabel('Time [s]');
ylabel('joint moments [Nm]');
legend('ankle','hip');

subplot(2,2,3)
plot(R.t,R.COM*100); hold on;
xlabel('Time [s]');
ylabel('COM position [cm]');

subplot(2,2,4)
plot(R.t,R.COP*100); hold on;
l = yline(P.BOS(1)*100); l.Color = [0 0 0];
l = yline(P.BOS(2)*100); l.Color = [0 0 0];
set(gca,'YLim',P.BOS*100*1.1);
xlabel('Time [s]');
ylabel('COP position [cm]');

%% visualise DIP

VisualiseDIP_model(R,P,2);