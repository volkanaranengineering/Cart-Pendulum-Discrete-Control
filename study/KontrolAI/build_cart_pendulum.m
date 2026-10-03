function build_cart_pendulum
% Elementary-block nonlinear plant. Compatible with MATLAB R2018b.
root=fileparts(mfilename('fullpath')); addpath(root);
mdl='cart_pendulum'; if bdIsLoaded(mdl), close_system(mdl,0); end
load_system('simulink'); new_system(mdl);
set_param(mdl,'Solver','ode45','RelTol','1e-8','AbsTol','1e-10', ...
    'MaxStep','0.005','StopTime','5','ReturnWorkspaceOutputs','on');
w=get_param(mdl,'ModelWorkspace');
names={'M','m','b','l','I','g','z0','force_ts'};
vals={0.5,0.2,0.1,0.3,0.006,9.8,[0;0;5*pi/180;0],[0 0;5 0]};
for k=1:numel(names), assignin(w,names{k},vals{k}); end
add_block('simulink/Sources/From Workspace',[mdl '/Force_N'],'VariableName','force_ts','Interpolate','off','OutputAfterFinalValue','Holding final value','Position',[40 95 155 125]);
p=[mdl '/Nonlinear_Plant']; add_block('built-in/Subsystem',p,'Position',[220 70 440 160]);
B(p,'In1','simulink/Ports & Subsystems/In1',[20 170 50 190]);
B(p,'xdot','simulink/Continuous/Integrator',[780 70 815 100],'InitialCondition','z0(2)');
B(p,'x','simulink/Continuous/Integrator',[860 70 895 100],'InitialCondition','z0(1)');
B(p,'omega','simulink/Continuous/Integrator',[780 330 815 360],'InitialCondition','z0(4)');
B(p,'theta','simulink/Continuous/Integrator',[860 330 895 360],'InitialCondition','z0(3)');
B(p,'sin_theta','simulink/Math Operations/Trigonometric Function',[70 340 115 370],'Operator','sin');
B(p,'cos_theta','simulink/Math Operations/Trigonometric Function',[70 440 115 470],'Operator','cos');
B(p,'friction','simulink/Math Operations/Gain',[95 90 140 120],'Gain','b');
B(p,'omega_squared','simulink/Math Operations/Product',[80 240 115 275],'Inputs','**');
B(p,'centrifugal_product','simulink/Math Operations/Product',[160 250 190 285],'Inputs','**');
B(p,'centrifugal','simulink/Math Operations/Gain',[220 250 260 280],'Gain','m*l');
B(p,'R','simulink/Math Operations/Sum',[300 155 330 205],'Inputs','+--');
B(p,'hcos','simulink/Math Operations/Gain',[160 440 205 470],'Gain','m*l');
B(p,'G','simulink/Math Operations/Gain',[160 340 205 370],'Gain','m*l*g');
B(p,'hcos_squared','simulink/Math Operations/Product',[265 440 300 475],'Inputs','**');
B(p,'mass_product','simulink/Sources/Constant',[260 520 345 550],'Value','(M+m)*(I+m*l^2)');
B(p,'Delta','simulink/Math Operations/Sum',[390 440 420 480],'Inputs','+-');
B(p,'JR','simulink/Math Operations/Gain',[385 120 435 150],'Gain','I+m*l^2');
B(p,'hcosG','simulink/Math Operations/Product',[390 230 425 265],'Inputs','**');
B(p,'hcosR','simulink/Math Operations/Product',[390 310 425 345],'Inputs','**');
B(p,'massG','simulink/Math Operations/Gain',[455 365 505 395],'Gain','M+m');
B(p,'x_numerator','simulink/Math Operations/Sum',[520 130 550 170],'Inputs','++');
B(p,'theta_numerator','simulink/Math Operations/Sum',[550 310 580 350],'Inputs','++');
B(p,'xddot','simulink/Math Operations/Product',[665 95 710 135],'Inputs','*/');
B(p,'thetaddot','simulink/Math Operations/Product',[665 325 710 365],'Inputs','*/');
B(p,'States','simulink/Signal Routing/Mux',[980 95 985 300],'Inputs','4');
B(p,'Out1','simulink/Ports & Subsystems/Out1',[1040 185 1070 205]);
edges={'In1/1','R/1';'xdot/1','friction/1';'friction/1','R/2'; ...
 'omega/1','omega_squared/1';'omega/1','omega_squared/2'; ...
 'omega_squared/1','centrifugal_product/1';'theta/1','sin_theta/1'; ...
 'theta/1','cos_theta/1';'sin_theta/1','centrifugal_product/2'; ...
 'centrifugal_product/1','centrifugal/1';'centrifugal/1','R/3'; ...
 'cos_theta/1','hcos/1';'sin_theta/1','G/1'; ...
 'hcos/1','hcos_squared/1';'hcos/1','hcos_squared/2'; ...
 'mass_product/1','Delta/1';'hcos_squared/1','Delta/2'; ...
 'R/1','JR/1';'hcos/1','hcosG/1';'G/1','hcosG/2'; ...
 'hcos/1','hcosR/1';'R/1','hcosR/2';'G/1','massG/1'; ...
 'JR/1','x_numerator/1';'hcosG/1','x_numerator/2'; ...
 'hcosR/1','theta_numerator/1';'massG/1','theta_numerator/2'; ...
 'x_numerator/1','xddot/1';'Delta/1','xddot/2'; ...
 'theta_numerator/1','thetaddot/1';'Delta/1','thetaddot/2'; ...
 'xddot/1','xdot/1';'xdot/1','x/1';'thetaddot/1','omega/1';'omega/1','theta/1'; ...
 'x/1','States/1';'xdot/1','States/2';'theta/1','States/3';'omega/1','States/4';'States/1','Out1/1'};
for k=1:size(edges,1), add_line(p,edges{k,1},edges{k,2},'autorouting','on'); end
add_block('simulink/Sinks/To Workspace',[mdl '/State_log'],'VariableName','states','SaveFormat','Structure With Time','Position',[520 65 635 95]);
add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[540 125 585 160]);
add_block('simulink/Ports & Subsystems/Out1',[mdl '/state'],'Position',[665 105 695 125]);
add_line(mdl,'Force_N/1','Nonlinear_Plant/1');
add_line(mdl,'Nonlinear_Plant/1','State_log/1','autorouting','on');
add_line(mdl,'Nonlinear_Plant/1','Scope/1','autorouting','on');
add_line(mdl,'Nonlinear_Plant/1','state/1','autorouting','on');
Simulink.Annotation(mdl,'Input: force [N]. Output: [x (m), v (m/s), theta (rad), omega (rad/s)]. theta=0 is upright.');
set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(root,[mdl '.slx']));
end
function B(p,n,lib,pos,varargin)
add_block(lib,[p '/' n],'Position',pos,varargin{:});
end

