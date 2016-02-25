

A = [];
b = [];
Aeq = [];
beq = [];

lb = [-1,-1,-1];
ub = [1,1,1];

x0 = [0,0,0];
x = fmincon(@ALV2FUNCTION,x0,A,b,Aeq,beq,lb,ub)