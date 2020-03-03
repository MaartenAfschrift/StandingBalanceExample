%% Construct EOM of a double IP in matlab

clear all; close all; clc;
% We use the matlab symbolic toolbox to create the EoM of a double IP
% model.

% q     = joint angles
% th    = segment angles with respect to vertical
% dth   = first time derivative of th
% ddth  = second time derivative of th
% m     = segment mass
% g     = gravity
% l     = segment length
% I     = segment inertia
% d     = location COM
% u     = joint torques (actuation)

% create symbolic variables
syms q1 q2 qd1 qd2 qdd1 qdd2 'real';
syms l m 'real';
syms ap

% model properties
g  = 9.81;		% gravity
m1 = 0.35*m;	% mass legs
m2 = 0.65*m;	% mass trunk
l1 = 0.5*l;		% length legs
l2 = 0.5*l;		% length trunk
d1 = l1.*0.5;	% COM legs
d2 = l2.*0.35;	% COM trunk
I1 = m1*l1^2/12;	% inertia legs
I2 = m2*l2^2/12;	% inertia trunk

% convert joint angles to absolute angles
th1     = q1+pi/2;  % w.r.t. vertical instead of horizontal axis
dth1    = qd1;
ddth1   = qdd1;

th2     = q2 + th1;
dth2    = qd2 + dth1;
ddth2   = qdd2 + ddth1;

% create unit vector and its time derivative
i = sym([1;0;0]);
j = sym([0;1;0]);
k = sym([0;0;1]);

e1 = cos(th1)*i + sin(th1)*j;  %Unit vector along first link
n1 = -sin(th1)*i + cos(th1)*j;  %Unit vector normal to first link
e2 = cos(th2)*i + sin(th2)*j;  %Unit vector along first link
n2 = -sin(th2)*i+ cos(th2)*j;  %Unit vector normal to first link

de1 = dth1*n1; %First time derivative of unit vector along first link
dn1 = -dth1*e1;
de2 = dth2*n2; %First time derivative of unit vector along second link
dn2 = -dth2*e2;

%Second derivatives
dde1 = ddth1*n1 + dth1*dn1;
ddn1 = -ddth1*e1 - dth1*de1;
dde2 = ddth2*n2 + dth2*dn2;
ddn2 = -ddth2*e2 - dth2*de2;

% get the COM, COM_dot, COM_ddot and joint position
g1 = d1*e1;     % Position of center of mass of first link
dg1 = d1*de1;   % Velocity of the center of mass of first link
ddg1 = d1*dde1; % Accelerationh of the CoM of the first link
p1 = l1*e1;     % Position of the first joint

g2 = l1*e1 + d2*e2;         % Position of the center of mass of the second link
dg2 = l1*de1 + d2*de2;      % Velocity of the center of mass of the second link
ddg2 = l1*dde1 + d2*dde2;   % Acceleration of the CoM of the second link
p2 = p1 + l2*e2;            % Position of the second joint

%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Lagrangian Definitions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Kinetic energy:
T1 = (1/2)*m1*dot(dg1,dg1) + (1/2)*I1*dth1*dth1;  %First link
T2 = (1/2)*m2*dot(dg2,dg2) + (1/2)*I2*dth2*dth2;  %Second link
T = T1 + T2;

%Potential energy:
yIndex = 2;
U1 = m1*g*g1(yIndex);
U2 = m2*g*g2(yIndex);
U = U1 + U2;

%Lagrangian:
L = T - U; 

%Generalized coordinates:
q = [q1, q2];
dq = [qd1, qd2];
ddq = [qdd1, qdd2];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  evaluate partial derivatives                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%              DL  
%  DL_Dq  ==  ---      Note that 'D' is partial derivative here 
%              Dq
%
DL_Dq = jacobian(L,q')';

%              DL  
%  DL_Ddq  ==  ---      Note that 'D' is partial derivative here 
%              Ddq
%
DL_Ddq = jacobian(L,dq);

%                D  / DL  \         * Note that some of those 'd' should be
% DDL_DtDdq  ==  -- | --  |         curvy 'D' to represent partial
%                Dt \ Ddq /         derivatives
%
DDL_DtDdq = jacobian(DL_Ddq',[q, dq]) * [dq, ddq]';

%Write out as single equation:
EoM = DL_Dq - DDL_DtDdq;

%% get the perturbation vector
% (support surface translation)

Pin = [m1*g1(2) + m2*g2(2);
      m2*g2(2)];

Pert = Pin.*ap;

%% equations actuated pendulum + perturbations
Tid = -EoM + Pert;
matlabFunction(Tid,'file','f_Tid.m');

%% Symbolic function for COM kinematics
COM    = (g1.*m1 + g2.*m2)./(m1 + m2);
COMd   = (dg1.*m1 + dg2.*m2)./(m1 + m2);
COMdd  = (ddg1.*m1 + ddg2.*m2)./(m1 + m2);

COMx = COM(1);
matlabFunction(COMx,'file',fullfile(pwd,'f_COMx.m'));
% matlabFunction(COM,'file',fullfile(pwd,'f_COM.m'));
% matlabFunction(COMd,'file',fullfile(pwd,'f_COMd.m'));
% matlabFunction(COMdd,'file',fullfile(pwd,'f_COMdd.m'));

%% approximation of COP position

% Ankle moment ~ COP*Fy
Fy = COMdd(yIndex).*(m1+m2) + (m1+m2).*g; 	% GRF - m.g = ma;
COP = Tid(1)./Fy;
matlabFunction(Fy,'file',fullfile(pwd,'f_Fy.m'));
matlabFunction(COP,'file',fullfile(pwd,'f_COP.m'));

%% Explicit equations of motion
syms u1 u2 'real';
EoMA = EoM - Pert + [u1; u2];   % actuated model
EoMA = simplify(EoMA);
save('EoMA.mat','EoMA');
% Qdd = solve(EqExpl,[qdd1 qdd2]);
% matlabFunction(Qdd1,'file',fullfile(pwd,'f_Qdd1.m'));
% matlabFunction(Qdd2,'file',fullfile(pwd,'f_Qdd2.m'));


