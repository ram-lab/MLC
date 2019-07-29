function [e_x, e_y, e_z, e_roll, e_pitch, e_yaw] = calculate_error(Tform_gt, Tform_est)
    e_x = abs(Tform_gt(1,4) - Tform_est(1,4));
    e_y = abs(Tform_gt(2,4) - Tform_est(2,4));
    e_z = abs(Tform_gt(3,4) - Tform_est(3,4));

    [yaw_gt, pitch_gt, roll_gt] = dcm2angle(Tform_gt(1:3, 1:3), 'ZYX');
    [yaw_est, pitch_est, roll_est] = dcm2angle(Tform_est(1:3, 1:3), 'ZYX');
    e_roll = abs(roll_gt - roll_est);
    e_pitch = abs(pitch_gt - pitch_est);
    e_yaw = abs(yaw_gt - yaw_est);
end