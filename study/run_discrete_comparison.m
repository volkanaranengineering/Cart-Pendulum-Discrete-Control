function results=run_discrete_comparison
% Run old controllers, isolate plant discretization, then increase control rate.
root=fileparts(mfilename('fullpath'));cd(root);addpath(root,fullfile(root,'KontrolAI'));
build_discrete_comparison;
folder=fullfile(root,'discrete_results');if ~exist(folder,'dir'),mkdir(folder);end
models={'cart_pendulum_staged_comparison','cart_pendulum_discrete_2ms','cart_pendulum_discrete'};
variants={'previous','discrete_2ms','discrete_01ms'};
labels={'PI','LQR','RLS_DOB','ESO','P_only','Fixed_inverse'};
t=(0:1e-4:20)';runs=cell(3,2,6);rows={};checks={};
for v=1:3
 mdl=models{v};load_system(mdl);w=get_param(mdl,'ModelWorkspace');
 for trial=1:2
  disturbance=[0 0;20 0];tag='baseline';
  if trial==2,disturbance=[0 0;8 1;8.1 0;20 0];tag='kick';end
  assignin(w,'disturbance_ts',disturbance);
  fprintf('Simulating %s %s\n',variants{v},tag);o=sim(mdl);
  for j=1:6
   s=o.get(labels{j});[tt,ii]=unique(s.time,'last');a=s.signals.values(ii,:);
   y=interp1(tt,a,t,'previous');
   if v==1,y(:,1:4)=interp1(tt,a(:,1:4),t,'linear');end
   assert(all(isfinite(y(:))),'Nonfinite trajectory');assert(max(abs(y(:,5)))<=10+1e-8);
   if v>1
    assert(numel(tt)==numel(t) && max(abs(tt-t))<1e-9,'Unexpected discrete logging grid');
   end
   runs{v,trial,j}=y;
   % Physical COM positions: positive theta leans LEFT in original equations.
   xy=cart_pendulum_xy(y(:,1:4),.3);
   headers={'t','x','v','theta','omega','u','raw','dhat','a_hat','h_hat','b_hat','innovation','updated','phi1','phi2','phi3','uf','omega_hat','f_hat','disturbance','cart_x','cart_y','bob_x','bob_y'};
   writetable(array2table([t y xy],'VariableNames',headers),fullfile(folder,[variants{v} '_' labels{j} '_' tag '.csv']));
  end
  for j=1:6
   y=runs{v,trial,j};startup=t<5;
   assert(max(max(abs(y(startup,1:5)-runs{v,trial,1}(startup,1:5))))<1e-7);
   assert(max(abs(y(startup,7)))==0);
  end
  y=runs{v,trial,3};frozen=t>=5.002;
  assert(max(max(abs(bsxfun(@minus,y(frozen,8:10),y(find(frozen,1),8:10)))))<1e-12);
  assert(all(y(frozen,12)==0));
 end
 for j=1:6
  y=runs{v,2,j};delta=y(:,1:4)-runs{v,1,j}(:,1:4);ix=t>=8;
  lost=find(abs(y(:,3))>=pi/2,1);tl=NaN;if ~isempty(lost),tl=t(lost);end
  rows(end+1,:)={variants{v},labels{j},max(abs(y(ix,3)))*180/pi,max(abs(delta(ix,3)))*180/pi,max(abs(delta(ix,1))),y(end,1),y(end,3)*180/pi,sqrt(trapz(t(ix),y(ix,5).^2)/12),settle(t,abs(delta(:,3))<pi/180,8.1),settle(t,abs(delta(:,1))<.01,8.1),tl}; %#ok<AGROW>
 end
end
summary=cell2table(rows,'VariableNames',{'variant','controller','postkick_peak_angle_deg','pulse_peak_angle_deg','pulse_peak_x_m','final_x_m','final_angle_deg','postkick_rms_force_N','pulse_angle_settle_s','pulse_x_settle_s','first90deg_s'});
writetable(summary,fullfile(folder,'summary.csv'));disp(summary);
for j=1:6
 a=runs{2,2,j}-runs{1,2,j};bb=runs{3,2,j}-runs{1,2,j};
 checks(end+1,:)={labels{j},max(abs(a(:,1))),max(abs(a(:,3)))*180/pi,max(abs(bb(:,1))),max(abs(bb(:,3)))*180/pi}; %#ok<AGROW>
end
differences=cell2table(checks,'VariableNames',{'controller','same_rate_max_x_error_m','same_rate_max_angle_error_deg','fast_rate_max_x_change_m','fast_rate_max_angle_change_deg'});
writetable(differences,fullfile(folder,'differences.csv'));
results=struct('t',t,'runs',{runs},'labels',{labels},'variants',{variants},'summary',summary,'differences',differences,'Ts',1e-4,'l',.3);
save(fullfile(folder,'results.mat'),'results','-v7.3');
% Independent RK4 / mass-matrix ODE and linearization checks.
validate_discrete_models(folder);
plot_cart_pendulum_results(results,folder);
animate_cart_pendulum(results,fullfile(folder,'controllers_2d.mp4'),4);
fprintf('Completed. Results: %s\n',folder);
end
function s=settle(t,ok,start)
i=find(~ok & t>=start,1,'last');if isempty(i),s=0;elseif i==numel(t)||t(i+1)>t(end)-1,s=NaN;else,s=t(i+1)-start;end
end
