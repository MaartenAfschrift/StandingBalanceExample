function out1 = f_Qdd1(ap,q1,q2,qd1,qd2,u1,u2)
%F_QDD1
%    OUT1 = F_QDD1(AP,Q1,Q2,QD1,QD2,U1,U2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Mar-2020 09:00:52

t2 = cos(q2);
t3 = sin(q2);
t4 = q1+q2;
t5 = cos(q1);
t6 = qd1.^2;
out1 = ((u1.*3.952e6-u2.*3.952e6+sin(q1).*1.903079178e9-ap.*t5.*4.11502e7+t3.*t6.*4.5470971e7-t2.*u2.*6.72e6+qd2.^2.*t3.*4.5470971e7-t2.*sin(t4).*8.92352916e8+ap.*t2.*cos(t4).*9.09636e7+ap.*t2.*t5.*2.59896e8+qd1.*qd2.*t3.*9.0941942e7+t2.*t3.*t6.*7.731906e7).*(-5.703638921632001e-6))./(t2.^2.*4.41e2-8.74e2);
