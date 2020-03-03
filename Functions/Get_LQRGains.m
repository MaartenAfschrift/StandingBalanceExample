function [K_LQR,LQR] = Get_LQRGains(m_in,l_in)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

import casadi.*;

% load EoM
load('EoMA.mat');

% create the symbolic variables again
syms m l
% subsitute model properties in equation
EOM_subs = subs(EoMA,[m l],[m_in, l_in]);
EOM_subs = simplify(EOM_subs);

% solve for the joint accelerations
syms qdd1 qdd2
Qdd = solve(EOM_subs,[qdd1 qdd2]);
matlabFunction(Qdd.qdd1,'file',fullfile(pwd,'Functions','f_Qdd1.m'));
matlabFunction(Qdd.qdd2,'file',fullfile(pwd,'Functions','f_Qdd2.m'));
clear qdd1 qdd2 EoMA EoM_subs Qdd


q   = MX.sym('q',2);  dq = MX.sym('dq',2);    T = MX.sym('dq',2);
ddq1 =  f_Qdd1(0,q(1),q(2),dq(1),dq(2),T(1),T(2));
ddq2 =  f_Qdd2(0,q(1),q(2),dq(1),dq(2),T(1),T(2));
ddq = [ddq1; ddq2];
xd  = [dq; ddq];
fcn_explicit = Function('fcn_det',{q, dq, T},{ddq});
Jc_f_q = jacobian(xd,q);    Jc_f_dq = jacobian(xd,dq);
Jc_f_x = [Jc_f_q  Jc_f_dq];
Jc_f_u = jacobian(xd,T);
Afcn = Function('Afcn',{q,dq,T},{Jc_f_x});
Bfcn = Function('Cfcn',{q,dq,T},{Jc_f_u});
A = Afcn([0;0],zeros(2,1),zeros(2,1));
B = Bfcn([0;0],zeros(2,1),zeros(2,1));
Q = eye(4);         R = eye(2);
[K,~,~] = lqr(full(A),full(B),Q,R);
K_LQR = -K;


LQR.K = K_LQR;
LQR.A = full(A);
LQR.B = full(B);
end

