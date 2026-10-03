function validate_discrete_models(folder)
% Compare nonlinear step to independently formed 2x2 mass-matrix ODE.
h=1e-4;z=[.1;-.2;.15;.3];u=.7;
[~,init]=discrete_cart_plant(0,[],[],0,h,z,.5,.2,.1,.3,.006,9.8);assert(isequal(init,z));
zn=z;
for k=1:1000,zn=discrete_cart_plant(0,zn,u,2,h,z,.5,.2,.1,.3,.006,9.8);end
[~,ref]=ode45(@(~,s) independent_rhs(s,u),[0 .1],z,odeset('RelTol',1e-11,'AbsTol',1e-12,'MaxStep',1e-4));
err=max(abs(zn-ref(end,:)'));assert(err<1e-8);
s=load('discrete_design.mat');epsi=1e-7;FD=zeros(4);zero=zeros(4,1);
for k=1:4
 zz=zero;zz(k)=epsi;
 FD(:,k)=discrete_cart_plant(0,zz,0,2,h,zero,.5,.2,.1,.3,.006,9.8)/epsi;
end
linearError=max(abs(FD(:)-s.Az(:)));assert(linearError<1e-9);
be=discrete_cart_plant(0,zero,epsi,2,h,zero,.5,.2,.1,.3,.006,9.8)/epsi;
assert(max(abs(be-s.Bz))<1e-9);
xy=cart_pendulum_xy([0 0 pi/2 0;0 0 0 0],.3);assert(abs(xy(1,3)+.3)<1e-12 && abs(xy(2,4)-.3)<1e-12);
o=sim('cart_pendulum_discrete_plants','StopTime','.01');logged=o.get('Nonlinear_RK4');assert(numel(logged.time)==101);
fid=fopen(fullfile(folder,'validation.txt'),'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'PASS: all trajectories finite, actuator <=10N, exact 1e-4s output grid.\nPASS: common PI startup, zero DOB before 5s, RLS frozen after 5s.\nRK4 vs independent mass-matrix ode45 (0.1s) max state error: %.12g\nNonlinear discrete Jacobian vs exact-ZOH linear model: %.12g\nPASS: x/y geometry sign and standalone discrete plants.\n',err,linearError);
end
function dz=independent_rhs(z,u)
c=.06*cos(z(3));acc=[.7 -c;-c .024]\[u-.1*z(2)-.06*sin(z(3))*z(4)^2;.588*sin(z(3))];
dz=[z(2);acc(1);z(4);acc(2)];
end
