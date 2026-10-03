function build_discrete_comparison
% Rebuild previous comparison plus fully discrete models, MATLAB R2018b.
root=fileparts(mfilename('fullpath'));addpath(root,fullfile(root,'KontrolAI'));cd(root);
build_cart_pendulum;
M=.5;m=.2;b=.1;l=.3;I=.006;g=9.8;J=I+m*l*l;h=m*l;q=(M+m)*J-h*h;
A=[0 1 0 0;0 -J*b/q h*h*g/q 0;0 0 0 1;0 -h*b/q (M+m)*h*g/q 0];
B=[0;J/q;0;h/q];Q=diag([10 1 100 1]);Ru=.1;
H=[A -B*B'/Ru;-Q -A'];[V,D]=eig(H);U=V(:,real(diag(D))<0);
P=real(U(5:8,:)/U(1:4,:));K=B'*P/Ru;
assert(norm(A'*P+P*A-P*B*B'*P/Ru+Q,'fro')<1e-7);
save(fullfile(root,'comparison_design.mat'),'A','B','K','Q','Ru','P');
[Ad,Bd]=observer(.002);save(fullfile(root,'eso_design.mat'),'Ad','Bd');
build_staged_comparison;
labels={'PI','LQR','RLS_DOB','ESO','P_only','Fixed_inverse'};
for variant=1:2
 name={'cart_pendulum_discrete_2ms','cart_pendulum_discrete'};mdl=name{variant};
 if bdIsLoaded(mdl),close_system(mdl,0);end
 load_system('cart_pendulum_staged_comparison');
 save_system('cart_pendulum_staged_comparison',fullfile(root,[mdl '.slx']));
 close_system('cart_pendulum_staged_comparison',0);load_system(mdl);
 Ts=1e-4;Tc=.002;if variant==2,Tc=Ts;end
 set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','1e-4');
 w=get_param(mdl,'ModelWorkspace');assignin(w,'Ts',Ts);assignin(w,'Tc',Tc);
 [Ad,Bd]=observer(Tc);assignin(w,'Ad',Ad);assignin(w,'Bd',Bd);
 set_param([mdl '/Kick'],'SampleTime','Ts');
 for j=1:6
  path=[mdl '/Plant_' labels{j}];pos=get_param(path,'Position');
  delete_line(mdl,['Sum_' labels{j} '/1'],['Plant_' labels{j} '/1']);
  delete_line(mdl,['Plant_' labels{j} '/1'],['Control_' labels{j} '/1']);
  delete_line(mdl,['Plant_' labels{j} '/1'],['Vector_' labels{j} '/1']);
  delete_block(path);
  add_block('simulink/User-Defined Functions/S-Function',path,'FunctionName','discrete_cart_plant', ...
   'Parameters','Ts,z0,M,m,b,l,I,g','Position',pos);
  % Reconnect the state loop after removing all original line branches.
  add_line(mdl,['Sum_' labels{j} '/1'],['Plant_' labels{j} '/1'],'autorouting','on');
  add_line(mdl,['Plant_' labels{j} '/1'],['Control_' labels{j} '/1'],'autorouting','on');
  add_line(mdl,['Plant_' labels{j} '/1'],['Vector_' labels{j} '/1'],'autorouting','on');
  set_param([mdl '/Control_' labels{j}],'FunctionName','discrete_staged_controller', ...
   'Parameters',sprintf('%d,K,Ad,Bd,Tc',j));
 end
 annotations=find_system(mdl,'FindAll','on','type','annotation');
 for k=1:numel(annotations),obj=get_param(annotations(k),'Object');delete(obj);end
 Simulink.Annotation(mdl,sprintf('Fully discrete RK4 plant, h=0.0001 s. Controller Ts=%g s. PI startup 0-5s, switch/freeze at 5s, pulse 8-8.1s. No continuous states.',Tc));
 set_param(mdl,'SimulationCommand','update');save_system(mdl);
end
% Standalone nonlinear and exact-ZOH linear discrete plants.
Ts=1e-4;E=expm([A B;zeros(1,5)]*Ts);Az=E(1:4,1:4);Bz=E(1:4,5);
save(fullfile(root,'discrete_design.mat'),'A','B','Az','Bz','K','Ts');
mdl='cart_pendulum_discrete_plants';if bdIsLoaded(mdl),close_system(mdl,0);end;new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','1e-4','StopTime','2','ReturnWorkspaceOutputs','on');
w=get_param(mdl,'ModelWorkspace');names={'Ts','z0','M','m','b','l','I','g','Az','Bz'};
values={Ts,[0;0;5*pi/180;0],M,m,b,l,I,g,Az,Bz};
for k=1:numel(names),assignin(w,names{k},values{k});end
add_block('simulink/Sources/Constant',[mdl '/Force_N'],'Value','0','SampleTime','Ts','Position',[30 80 80 110]);
add_block('simulink/User-Defined Functions/S-Function',[mdl '/Nonlinear_RK4'],'FunctionName','discrete_cart_plant','Parameters','Ts,z0,M,m,b,l,I,g','Position',[160 40 330 90]);
add_block('simulink/Discrete/Discrete State-Space',[mdl '/Linear_ZOH'],'A','Az','B','Bz','C','eye(4)','D','zeros(4,1)','InitialCondition','z0','SampleTime','Ts','Position',[160 150 330 200]);
plants={'Nonlinear_RK4','Linear_ZOH'};
for k=1:2
 add_block('simulink/Sinks/To Workspace',[mdl '/' plants{k} '_log'],'VariableName',plants{k},'SaveFormat','Structure With Time','Position',[400 40+110*(k-1) 520 70+110*(k-1)]);
 add_line(mdl,'Force_N/1',[plants{k} '/1']);add_line(mdl,[plants{k} '/1'],[plants{k} '_log/1']);
end
Simulink.Annotation(mdl,'State order [x v theta omega]. Linear ZOH valid near upright; nonlinear RK4 supports large angles. theta=0 upright. Fixed step 1e-4 s.');
set_param(mdl,'SimulationCommand','update');save_system(mdl,fullfile(root,[mdl '.slx']));
end
function [Ad,Bd]=observer(Ts)
wo=20;A=[-3*wo 1 0;-3*wo^2 0 1;-wo^3 0 0];B=[3*wo 0;3*wo^2 1;wo^3 0];
E=expm([A B;zeros(2,5)]*Ts);Ad=E(1:3,1:3);Bd=E(1:3,4:5);
end
