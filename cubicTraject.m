function [a,b,c,d]=cubicTraject(x0,dx0,xf,dxf,t0,tf)

X=[x0;dx0;xf;dxf];

T=[t0^3,t0^2,t0,1;
    3*t0^2,2*t0,1,0;
    tf^3,tf^2,tf,1;
    3*tf^2,2*tf,1,0];


A=inv(T)*X;
a=A(1);
b=A(2);
c=A(3);
d=A(4);

end