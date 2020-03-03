%% Information about functions

% We create the following matlab functions in the folder EoM (using the
% script DoublePendulum_EOM_Lagrange.m)
%    1) f_Tid.m: solve inerse dynamics of the double IP
%    2) f_COP.m: location center of pressure
%    2) f_COM, f_COMd, f_COMdd: COM position, velocity and acceleration.
%
% In short, the function f_Tid solves the inverse dynamics problem of
% the double inverted pendulum.
% Input variables: f_EoM_DIP
%   Model properties described above: l and m
%   acceleration of the platform (i.e. perturbation force) = ap
%   joint angles: q1,q2
%   joint velocities: qd1, qd2
%   joint accelerations: qdd1, qdd2:
% Output: ankle and hip/lumbar joint moment

% the functions f_COM and f_COMd use similar input argument and compute the
% horizontal and vertical position of the COM.

% example: joint moments needed to keep static upright position when
% platform acceleration 1 m/s2 forward
q1 = 5*pi./180; q2 =0; % joint angles (in radians)
qd1 =0; qd2 = 0;        % joint velocities  (in radians/s)
qdd1= 0; qdd2 = 0;      % joint accelerations  (in radians/s2)
ap = 0;                 % forward surface acceleration [m/s2]

% joint moments
T = f_Tid(ap,P.l,P.m,q1,q2,qd1,qd2,qdd1,qdd2);

% COM position
COMx = f_COMx(P.l,P.m,q1,q2);

% vertical ground reaction force and COP position
Fy  = f_Fy(P.l,P.m,q1,q2,qd1,qd2,qdd1,qdd2);
COP = f_COP(ap,P.l,P.m,q1,q2,qd1,qd2,qdd1,qdd2);