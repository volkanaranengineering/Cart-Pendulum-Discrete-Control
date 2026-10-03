function figs=plot_cart_pendulum_results(results,folder)
% Reusable plotter. Input is results struct or saved results.mat path.
if ischar(results),s=load(results,'results');results=s.results;end
if nargin<2,folder=pwd;end
if ~exist(folder,'dir'),mkdir(folder);end
t=results.t;ii=1:20:numel(t);styles={'--',':','-'};figs=gobjects(3,1);
figs(1)=figure('Visible','off','Color','w','Position',[50 50 1500 850]);
for j=1:6
 subplot(3,2,j);hold on;
 for v=1:3,y=results.runs{v,2,j};plot(t(ii),y(ii,3)*180/pi,styles{v},'LineWidth',1.1);end
 grid on;xlabel('Time (s)');ylabel('Angle (deg)');title(strrep(results.labels{j},'_',' '));
 if j==1,legend('Previous: continuous plant, 2 ms control','Discrete plant, 2 ms control','Discrete plant, 0.1 ms control','Location','best');end
end
print(figs(1),fullfile(folder,'angle_comparison.png'),'-dpng','-r130');savefig(figs(1),fullfile(folder,'angle_comparison.fig'));
figs(2)=figure('Visible','off','Color','w','Position',[50 50 1500 850]);
for j=1:6
 subplot(3,2,j);hold on;
 for v=1:3,y=results.runs{v,2,j};plot(t(ii),y(ii,1),styles{v},'LineWidth',1.1);end
 grid on;xlabel('Time (s)');ylabel('Cart x (m)');title(strrep(results.labels{j},'_',' '));
 if j==1,legend('Previous','Discrete / 2 ms','Discrete / 0.1 ms','Location','best');end
end
print(figs(2),fullfile(folder,'position_comparison.png'),'-dpng','-r130');savefig(figs(2),fullfile(folder,'position_comparison.fig'));
figs(3)=figure('Visible','off','Color','w','Position',[50 50 1300 850]);
cols=lines(6);
for j=1:6
 y=results.runs{3,2,j};delta=y-results.runs{3,1,j};
 subplot(2,2,1);hold on;plot(t(ii),y(ii,5),'Color',cols(j,:));ylabel('Control force (N)');
 subplot(2,2,2);hold on;plot(t(ii),delta(ii,3)*180/pi,'Color',cols(j,:));ylabel('Pulse-only angle change (deg)');xlim([7.5 20]);
 subplot(2,2,3);hold on;plot(t(ii),delta(ii,1),'Color',cols(j,:));ylabel('Pulse-only cart change (m)');xlim([7.5 20]);
 subplot(2,2,4);hold on;xy=cart_pendulum_xy(y,.3);plot(xy(ii,3),xy(ii,4),'Color',cols(j,:));xlabel('Pendulum COM x (m)');ylabel('Pendulum COM y (m)');
end
for k=1:4,subplot(2,2,k);grid on;if k<4,xlabel('Time (s)');end;end
subplot(2,2,1);legend(strrep(results.labels,'_',' '),'Location','best');title('Fully discrete controllers, Ts = 0.0001 s');
print(figs(3),fullfile(folder,'discrete_response.png'),'-dpng','-r130');savefig(figs(3),fullfile(folder,'discrete_response.fig'));
end
