function [sys,x0,str,ts]=discrete_cart_plant(~,x,u,flag,Ts,z0,M,m,b,l,I,g)
% Nonlinear discrete state transition: RK4, zero-order-held force.
% State [cart x; cart velocity; angle from upright; angular velocity].
str=[];x0=[];ts=[];
switch flag
 case 0
  s=simsizes;s.NumContStates=0;s.NumDiscStates=4;s.NumOutputs=4;
  s.NumInputs=1;s.DirFeedthrough=0;s.NumSampleTimes=1;
  sys=simsizes(s);x0=z0;ts=[Ts 0];
 case 2
  a=rhs(x,u,M,m,b,l,I,g);bb=rhs(x+Ts*a/2,u,M,m,b,l,I,g);
  c=rhs(x+Ts*bb/2,u,M,m,b,l,I,g);d=rhs(x+Ts*c,u,M,m,b,l,I,g);
  sys=x+Ts*(a+2*bb+2*c+d)/6;
 case 3
  sys=x;
 case {1,4,9}
  sys=[];
end
end
function dz=rhs(z,u,M,m,b,l,I,g)
J=I+m*l*l;h=m*l;co=cos(z(3));si=sin(z(3));
R=u-b*z(2)-h*si*z(4)^2;G=h*g*si;delta=(M+m)*J-(h*co)^2;
dz=[z(2);(J*R+h*co*G)/delta;z(4);(h*co*R+(M+m)*G)/delta];
end
