function visualization_sensor_odom(sensor_data_1,sensor_data_2,sensor_data_3,sensor_data_4)
    figure;
%     axis equal;
    sensor_data = sensor_data_1;
    x = reshape(sensor_data.tformAbs_world(1, 4, 1:500), [1,length(sensor_data.tformAbs_world(1, 4, 1:500))]);
    y = reshape(sensor_data.tformAbs_world(2, 4, 1:500), [1,length(sensor_data.tformAbs_world(2, 4, 1:500))]);
    z = reshape(sensor_data.tformAbs_world(3, 4, 1:500), [1,length(sensor_data.tformAbs_world(3, 4, 1:500))]);
    plot(x, y, '-', 'LineWidth', 5, 'Color', [228,26,28]/255 ); hold on;
    a = x(1); b = y(1);
    
    sensor_data = sensor_data_2;
    x = reshape(sensor_data.tformAbs_world(1, 4, 1:700), [1,length(sensor_data.tformAbs_world(1, 4, 1:700))]);
    y = reshape(sensor_data.tformAbs_world(2, 4, 1:700), [1,length(sensor_data.tformAbs_world(2, 4, 1:700))]);
    z = reshape(sensor_data.tformAbs_world(3, 4, 1:700), [1,length(sensor_data.tformAbs_world(3, 4, 1:700))]);
    plot(x, y, '-', 'LineWidth', 5, 'Color', [55,126,184]/255 ); hold on;
%     plot(x, y, '>', 'MarkerSize', 7, 'Color', [55,126,184]/255 ); hold on;  
%     plot(x(1), y(1), 'o', 'MarkerSize', 8, 'Color', [0,0,0]/255, 'MarkerFaceColor', [0,0,0]/255); hold on;
    c = x(1); d = y(1);    

%     sensor_data = sensor_data_3;
%     x = reshape(sensor_data.tformAbs_world(1, 4, :), [1,length(sensor_data.tformAbs_world(1, 4, :))]);
%     y = reshape(sensor_data.tformAbs_world(2, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
%     z = reshape(sensor_data.tformAbs_world(3, 4, :), [1,length(sensor_data.tformAbs_world(3, 4, :))]);
%     plot(x, y, '-', 'LineWidth', 2, 'Color', [77,175,74]/255 ); hold on;
%     plot(x, y, '>', 'MarkerSize', 7, 'Color', [77,175,74]/255 ); hold on;    
    
    sensor_data = sensor_data_4;
    x = reshape(sensor_data.tformAbs_world(1, 4, 50:322), [1,length(sensor_data.tformAbs_world(1, 4, 50:322))]);
    y = reshape(sensor_data.tformAbs_world(2, 4, 50:322), [1,length(sensor_data.tformAbs_world(2, 4, 50:322))]);
    z = reshape(sensor_data.tformAbs_world(3, 4, 50:322), [1,length(sensor_data.tformAbs_world(3, 4, 50:322))]);
    plot(x, y, '-', 'LineWidth', 5, 'Color', [152,78,163]/255 ); hold on;
%     plot(x, y, '>', 'MarkerSize', 7, 'Color', [152,78,163]/255 ); hold on;   
    e = x(1); f = y(1);
    
    plot(a, b, 'o', 'MarkerSize', 10, 'Color', [0,0,0]/255, 'MarkerFaceColor', [0,0,0]/255); hold on;
    plot(c, d, 'o', 'MarkerSize', 10, 'Color', [0,0,0]/255, 'MarkerFaceColor', [0,0,0]/255); hold on;
    plot(e, f, 'o', 'MarkerSize', 10, 'Color', [0,0,0]/255, 'MarkerFaceColor', [0,0,0]/255); hold on;
    
    grid on;          
%     axis off;
    xlabel('X [m]');
    ylabel('Y [m]');
    legend(' S.T 1', ' S.T 2', ' S.T 3', ' Start Location', 'Location','southwest');
%     legend boxoff;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 40);   
    xlim([-180, 40]);
    ylim([-250, 130]);
    ax = gca;
    ax.LineWidth = 4;
end

% 228,26,28
% 55,126,184
% 77,175,74
% 152,78,163