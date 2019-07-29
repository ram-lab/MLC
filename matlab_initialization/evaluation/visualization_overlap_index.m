% visualize the overlapping omega
%%
file = strcat('../data/evaluation/overlap_0_omega.txt');
[data] = textread(file, '%n%*[^\n]');
v_omega = [];
for i = 1:length(data)
    if ~mod(i,2)
        v_omega = [v_omega; data(i)];
    end
end

[max_ratio, index] = max(v_omega);
for i_start = index-1:-1:0
    if v_omega(i_start) < max_ratio*0.6
        break;
    end
end
i_start = i_start + 1;
for i_end = index+1:1:length(v_omega)
    if v_omega(i_end) < max_ratio*0.6
        break;
    end
end
i_end = i_end - 1;

figure;
title('Overlapping Estimation'); hold on; plot(1:1:length(v_omega), v_omega, '-', 'Color', [190,10,50]/255, 'LineWidth', 5); hold off;
ax = gca;
set(ax, 'FontName', 'Times New Roman', 'FontSize', 40);
% set(gca,'position',[0.1,0.2,0.35,0.45] );
xlabel('Timestamp');
ylabel('\Omega');
ax.YGrid = 'on';
ax.XGrid = 'on';
ylim([0, 0.018]);
% grid on;

figure;
hold on; plot(i_start:i_end, v_omega(i_start:i_end), '-', 'Color', [0, 0, 153]/255, 'LineWidth', 5); hold off;
ax = gca;
set(ax, 'FontName', 'Times New Roman', 'FontSize', 40);
% set(gca,'position',[0.1,0.2,0.35,0.45] );
% xlabel('Timestamp');
% ylabel('\Omega');
ax.YGrid = 'on';
ax.XGrid = 'on';
ylim([0, 0.018]);
% grid on;

