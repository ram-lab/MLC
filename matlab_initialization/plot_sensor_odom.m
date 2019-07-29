function plot_sensor_odom(sensor_data, comment, num, k)
    x = reshape(sensor_data.tformAbs_world(1, 4, :), [1,length(sensor_data.tformAbs_world(1, 4, :))]);
    y = reshape(sensor_data.tformAbs_world(2, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
    z = reshape(sensor_data.tformAbs_world(3, 4, :), [1,length(sensor_data.tformAbs_world(2, 4, :))]);
    subplot(2, num, k); plot(z, x, 'r-'); title(['Trajectory in World']); axis equal; hold on;
    plot(0, 0, 'ko', 'MarkerSize', 10);
    xlabel('Z (m)'); ylabel('X (m)'); zlabel('Y (m)');
    set(gca,'FontSize',10);
%     legend('Trajectory', 'Starting point', 'FontSize', 10);
    grid on;

    x = reshape(sensor_data.tformAbs_lidar(1, 4, :), [1,length(sensor_data.tformAbs_lidar(1, 4, :))]);
    y = reshape(sensor_data.tformAbs_lidar(2, 4, :), [1,length(sensor_data.tformAbs_lidar(2, 4, :))]);            
    z = reshape(sensor_data.tformAbs_lidar(3, 4, :), [1,length(sensor_data.tformAbs_lidar(2, 4, :))]);
    subplot(2, num, num+k); plot(x, y, 'b-'); title(comment); axis equal; hold on;
    plot(0, 0, 'ko','MarkerSize', 10);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    set(gca,'FontSize',10);
%     legend('Trajectory', 'Starting point', 'FontSize', 10);
    grid on;
end
