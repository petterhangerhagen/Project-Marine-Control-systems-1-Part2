
%% Plot 1
fig = figure;

%% Subplot 1 Surge
subplot(3,1,1);
hold on;
plot(tout,pos.data(:,1),"k",LineWidth=1,Displayname="North position")
%plot(tout,set_point.data(:,1),"k--",LineWidth=1,Displayname="reference"); 
grid on;
ylim([min(pos.data(:,1)) - 0.1, max(pos.data(:,1)) + 0.1]);
xlabel('time [s]','Interpreter','latex');
ylabel('North pos [m]','Interpreter','latex');
title('North position against reference postion','Interpreter','latex');
legend;
hold off;

%% Subplot 2 Sway
subplot(3,1,2);
hold on;
plot(tout,pos.data(:,2),"k",LineWidth=1,Displayname="East position")
%plot(tout,set_point.data(:,2),"k--",LineWidth=1,Displayname="reference"); 
grid on; 
ylim([min(pos.data(:,2)) - 0.1, max(pos.data(:,2)) + 0.1]);
xlabel('time [s]','Interpreter','latex');
ylabel('East pos [m]','Interpreter','latex');
title('East position against reference postion','Interpreter','latex');
legend;
hold off;

%% Subplot 3 Yaw
subplot(3,1,3);
hold on;
plot(tout,pos.data(:,3),"k",LineWidth=1,Displayname="Heading angle");
%plot(tout,set_point.data(:,3),"k--",LineWidth=1,Displayname="reference"); 
grid on;
ylim([min(pos.data(:,3)) - 0.1, max(pos.data(:,3)) + 0.1]);
xlabel('time [s]','Interpreter','latex');
ylabel('Heading angle [degrees]','Interpreter','latex');
title('Heading angle against reference heading angle','Interpreter','latex');
legend;
hold off;

%% Save plot 1
counter = 1;
savepath = strcat("Plots/plot_simulation",num2str(counter),".png");
saveas(fig,savepath);


%% Plot 2

fig2 = figure;

hold on;
plot(pos.data(:,2),pos.data(:,1),"k",LineWidth=1,Displayname="position"); 
%plot(set_point.data(:,2),set_point.data(:,1),"k--",LineWidth=1,Displayname="reference");
pos_every_10 = pos.data(100:100:end, :);
[yWind,xWind] = pol2cart((pi/180)*pos_every_10(:,3),Uz); 
quiver(pos_every_10(:,2),pos_every_10(:,1), xWind, yWind,0.4,DisplayName="Heading",color=[.5 .5 .5])
grid on;
xlabel("East pos [m]",'Interpreter','latex');
ylabel("North pos [m]",'Interpreter','latex');
title("xy-plot of position",'Interpreter','latex');
legend;
hold off;

%% Save plot 2
savepath = strcat("Plots/xy_plot_simulation",num2str(counter),".png");
saveas(fig2,savepath);


%%
% dims = [200 200 500 400];
% set(fig,'renderer','painters','position',dims,'PaperPositionMode','auto');
% savepath = "/Users/petterhangerhagen/Documents/MATLAB/Marine reguleringssystemer 1/Project part1/Plots/";
% print('-dpng','-r600',savepath + "plot1");




% 
% fig3 = figure;
% hold on;
% plot(tout,kp_out.data(:,1),"Blue",tout,ki_out.data(:,1),"Green",tout,kd_out.data(:,1),"Yellow");
% legend("P","I","D");
% grid on;
% hold off;


% %% saveing figures
% dims = [200 200 500 300]; % [x_pos, y_pos, x_brd, y_brd]
% set(fig,'renderer','painters','position',dims,'PaperPositionMode','auto');
% save_path = strcat('/Users/petterhangerhagen/Documents/MATLAB/Lineær systemteori/project/Plots/plot',num2str(counter));
% print('-dpng','-r600',save_path);

