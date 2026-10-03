function build_staged_comparison
root=fileparts(mfilename('fullpath'));cd(root);addpath(root);load_system('cart_pendulum');
mdl='cart_pendulum_staged_comparison';if bdIsLoaded(mdl),close_system(mdl,0);end;new_system(mdl);
set_param(mdl,'Solver','ode45','RelTol','1e-8','AbsTol','1e-10','MaxStep','.001','StopTime','20','ReturnWorkspaceOutputs','on');
w=get_param(mdl,'ModelWorkspace');names={'M','m','b','l','I','g','z0','disturbance_ts'};values={.5,.2,.1,.3,.006,9.8,[0;0;5*pi/180;0],[0 0;8 1;8.1 0;20 0]};
for k=1:numel(names),assignin(w,names{k},values{k});end
load('comparison_design.mat','K');load('eso_design.mat','Ad','Bd');assignin(w,'K',K);assignin(w,'Ad',Ad);assignin(w,'Bd',Bd);
add_block('simulink/Sources/From Workspace',[mdl '/Kick'],'VariableName','disturbance_ts','Interpolate','off','OutputAfterFinalValue','Holding final value','Position',[30 30 160 60]);
labels={'PI','LQR','RLS_DOB','ESO','P_only','Fixed_inverse'};
for k=1:6
 y=100+180*(k-1);p=['Plant_' labels{k}];c=['Control_' labels{k}];f=['Force_' labels{k}];s=['Sum_' labels{k}];v=['Vector_' labels{k}];lg=['Log_' labels{k}];
 add_block('cart_pendulum/Nonlinear_Plant',[mdl '/' p],'Position',[570 y 740 y+70]);
 add_block('simulink/User-Defined Functions/S-Function',[mdl '/' c],'FunctionName','staged_controller','Parameters',sprintf('%d,K,Ad,Bd',k),'Position',[200 y 350 y+65]);
 add_block('simulink/Math Operations/Gain',[mdl '/' f],'Gain','[1 zeros(1,13)]','Multiplication','Matrix(K*u)','Position',[390 y 455 y+40]);
 add_block('simulink/Math Operations/Sum',[mdl '/' s],'Inputs','++','Position',[500 y 530 y+40]);
 add_block('simulink/Signal Routing/Mux',[mdl '/' v],'Inputs','3','Position',[800 y 805 y+95]);
 add_block('simulink/Sinks/To Workspace',[mdl '/' lg],'VariableName',labels{k},'SaveFormat','Structure With Time','Position',[870 y 960 y+40]);
 edges={[p '/1'],[c '/1'];[c '/1'],[f '/1'];[f '/1'],[s '/1'];'Kick/1',[s '/2'];[s '/1'],[p '/1'];[p '/1'],[v '/1'];[c '/1'],[v '/2'];'Kick/1',[v '/3'];[v '/1'],[lg '/1']};
 for j=1:size(edges,1),add_line(mdl,edges{j,1},edges{j,2},'autorouting','on');end
end
Simulink.Annotation(mdl,'0-5s: identical PI, RLS learns inverse from clean transient. 5s: freeze RLS and switch controllers / enable DOB. 8-8.1s: 1N kick. All control samples 2ms, one-sample causal hold.');
set_param(mdl,'SimulationCommand','update');save_system(mdl,fullfile(root,[mdl '.slx']));
end
