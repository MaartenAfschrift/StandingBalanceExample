function COMx = f_COMx(l,m,q1,q2)
%F_COMX
%    COMX = F_COMX(L,M,Q1,Q2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Mar-2020 11:34:26

t2 = pi./2.0;
t3 = q1+t2;
t4 = cos(t3);
COMx = (m.*((l.*t4)./2.0+l.*cos(q1+q2+t2).*(7.0./4.0e1)).*(1.3e1./2.0e1)+l.*m.*t4.*(7.0./8.0e1))./m;
