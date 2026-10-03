function [sys,x0,str,ts]=staged_controller(t,x,z,flag,mode,K,Ad,Bd)
% Six branches share PI startup; training uses actuator force only before 5 s.
Ts=.002;str=[];x0=[];ts=[];
switch flag
case 0
 s=simsizes;s.NumContStates=0;s.NumDiscStates=32;s.NumOutputs=14;s.NumInputs=4;s.DirFeedthrough=0;s.NumSampleTimes=1;
 sys=simsizes(s);x0=zeros(32,1);x0(10:12)=[.84;.048;.12];P=diag([1 .01 1]);x0(13:21)=P(:);ts=[Ts 0];
case 2
 n=x;par=x(10:12);P=reshape(x(13:21),3,3);ph=x(6:8);uf=x(9);err=0;updated=0;
 if t>Ts/2
  zm=(z+x(1:4))/2;ac=(z([2 4])-x([2 4]))/Ts;
  phi=[ac(1);-cos(zm(3))*ac(2)+sin(zm(3))*zm(4)^2;zm(2)];
  af=exp(-Ts/.02);ph=af*ph+(1-af)*phi;uf=af*uf+(1-af)*x(5);err=uf-ph'*par;
  if mode==3 && t<5-1e-9
   L=P*ph/(.9995+ph'*P*ph);par=par+L*err;P=(P-L*ph'*P)/.9995;P=(P+P')/2;updated=1;
  end
 end
 dh=0;obs=x(27:29);integ=x(26);
 if t<5-1e-9
  raw=-20*z(3)+integ;force=max(-10,min(10,raw));integ=integ+Ts*(-z(3)+force-raw);
 else
  if mode==3 || mode==6
   dh=max(-5,min(5,exp(-Ts/.05)*x(22)+(1-exp(-Ts/.05))*(ph'*par-uf)));
  elseif mode==4
   if t<5+Ts/2,obs=[z(3);0;0];else,obs=Ad*obs+Bd*[z(3);x(5)];end
   dh=max(-5,min(5,obs(3)));
  end
  if mode==1
   raw=-20*z(3)+integ;
  elseif mode==2
   raw=-K*z;
  else
   raw=-20*z(3)-dh;
  end
  force=max(-10,min(10,raw));
  if mode==1,integ=integ+Ts*(-z(3)+force-raw);end
 end
 n(1:4)=z;n(5)=force;n(6:8)=ph;n(9)=uf;n(10:12)=par;n(13:21)=P(:);
 n(22:26)=[dh;raw;err;updated;integ];n(27:29)=obs;n(30:32)=ph;sys=n;
case 3
 sys=[x(5);x(23);x(22);x(10:12);x(24:25);x(30:32);x(9);x(28:29)];
case {1,4,9}
 sys=[];
end
end
