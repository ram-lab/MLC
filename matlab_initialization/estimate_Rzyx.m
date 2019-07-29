%% [roll pitch yaw] calibration on non-planner movement
function R_zyx = estimate_Rzyx(Tform_base, Tform_ref)
    A = zeros(4*length(Tform_base), 4); % 4N*4
    for j = 1:length(Tform_base)
        q_base = rotm2quat(Tform_base(1:3,1:3, j));
        w = q_base(1); x = q_base(2); y = q_base(3); z = q_base(4); 
        Q_base = [w -x -y -z; ... % left
                  x w -z y; ...
                  y z w -x; ...
                  z -y x w];

        q_rel = rotm2quat(Tform_ref(1:3,1:3, j));
        w = q_rel(1); x = q_rel(2); y = q_rel(3); z = q_rel(4);
        Q_rel = [w -x -y -z; ...
                 x w z -y; ...
                 y -z w x; ...
                 z y -x w];
        A(4*(j-1)+1:4*j, :) = Q_base - Q_rel;
    end
    [U, S, V] = svd(A, 'econ');
    q_zyx = V(:, end)';    
    R_zyx = quat2rotm(q_zyx);
end