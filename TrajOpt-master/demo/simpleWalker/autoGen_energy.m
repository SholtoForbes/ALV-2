function [U,T] = autoGen_energy(q1,q2,dq1,dq2,d,m,I,g,l)
%AUTOGEN_ENERGY
%    [U,T] = AUTOGEN_ENERGY(Q1,Q2,DQ1,DQ2,D,M,I,G,L)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    07-Sep-2015 19:06:40

t2 = cos(q1);
t3 = dq1.^2;
t4 = d.*t2;
t5 = l.*t2;
t6 = t4-t5;
t7 = sin(q1);
t8 = d.*t7-l.*t7;
t9 = cos(q2);
U = g.*m.*(t5-d.*t9)-g.*m.*(t4-l.*t2);
if nargout > 1
    t10 = d.*dq2.*t9-dq1.*l.*t2;
    t11 = d.*dq2.*sin(q2)-dq1.*l.*t7;
    T = I.*t3.*(1.0./2.0)+I.*dq2.^2.*(1.0./2.0)+m.*(t3.*t6.^2+t3.*t8.^2).*(1.0./2.0)+m.*(t10.^2+t11.^2).*(1.0./2.0);
end