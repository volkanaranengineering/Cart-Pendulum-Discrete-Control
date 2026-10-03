function fig=animate_cart_pendulum(results,videoPath,speed)
% MATLAB x/y animation, six panels. Tracks cart with a local moving viewport.
% speed: simulation seconds per video second. Empty path = interactive only.
if ischar(results),s=load(results,'results');results=s.results;end
if nargin<2,videoPath='';end;if nargin<3,speed=1;end
assert(isscalar(speed)&&isfinite(speed)&&speed>0);
fps=30;frameTimes=0:speed/fps:results.t(end);frameTimes=unique([frameTimes results.t(end)]);
fig=figure('Color','w','Position',[40 40 1280 900],'Name','Discrete cart-pendulum controller results');
ax=gobjects(6,1);cart=gobjects(6,1);rod=gobjects(6,1);bob=gobjects(6,1);trail=gobjects(6,1);txt=gobjects(6,1);xy=cell(6,1);
for j=1:6
 ax(j)=subplot(3,2,j);hold on;axis equal;grid on;ylim([-.22 .45]);
 xlabel('World x (m), moving view');ylabel('World y (m)');title(strrep(results.labels{j},'_',' '));
 plot([-1000 1000],[-.09 -.09],'k-');
 cart(j)=patch([-.1 .1 .1 -.1],[-.08 -.08 0 0],[.25 .55 .85]);
 rod(j)=plot([0 0],[0 .3],'-','Color',[.15 .15 .15],'LineWidth',3);
 bob(j)=plot(0,.3,'o','MarkerFaceColor',[.95 .4 .15],'MarkerSize',10);
 trail(j)=plot(NaN,NaN,':','Color',[.7 .4 .2]);
 txt(j)=text(.02,.92,'','Units','normalized','FontSize',9);
 xy{j}=cart_pendulum_xy(results.runs{3,2,j},results.l);
end
if ~isempty(videoPath)
 writer=VideoWriter(videoPath,'MPEG-4');writer.FrameRate=fps;open(writer);cleanup=onCleanup(@()close(writer)); %#ok<NASGU>
end
for tm=frameTimes
 [~,i]=min(abs(results.t-tm));
 for j=1:6
  p=xy{j}(i,:);y=results.runs{3,2,j};
  set(cart(j),'XData',p(1)+[-.1 .1 .1 -.1]);
  set(rod(j),'XData',p([1 3]),'YData',p([2 4]));set(bob(j),'XData',p(3),'YData',p(4));
  tail=max(1,i-round(.5/results.Ts)):100:i;set(trail(j),'XData',xy{j}(tail,3),'YData',xy{j}(tail,4));
  xlim(ax(j),p(1)+[-.55 .55]);
  set(txt(j),'String',sprintf('t=%.2f s | x=%.3f m | angle=%.2f deg\nu=%.2f N | disturbance=%.1f N',tm,p(1),y(i,3)*180/pi,y(i,5),y(i,19)));
 end
 drawnow;
 if ~isempty(videoPath),writeVideo(writer,getframe(fig));else,pause(1/fps);end
 if tm==frameTimes(1) && ~isempty(videoPath),saveas(fig,fullfile(fileparts(videoPath),'animation_preview.png'));end
end
end
