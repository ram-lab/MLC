%% [roll pitch] calibration on non-planner movement
function [b_solve, b_converaged, R_yx] = estimate_Ryx(Tform_base, Tform_ref)
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
%     disp(S);
    sigma_2 = S(3,3);
    v1 = V(:, 3);
    v2 = V(:, 4);
    
    % solve constraint for q_yz: xy = -zw
    [b_equ, s1, s2] = solveQuadraticEquation(v1(1) * v1(4) + v1(2) * v1(3), ...
                                            v1(2) * v2(3) + v1(3) * v2(2) + v1(4) * v2(1) + v1(1) * v2(4), ...
                                            v2(1) * v2(4) + v2(2) * v2(3));
    s = [s1; s2];
    if (~b_equ)
        print('# ERROR: Quadratic equation cannot be solved due to negative determinant. \n');
        b_solve = false;
        return
    end
    
    R_yxs = zeros(3,3,2);
    eul = zeros(2,3);
    for i=1:2
        t = s(i)*s(i)*dot(v1,v1) + 2*s(i)*dot(v1,v2) + dot(v2,v2);
        
        % solve constraint ||q_yx|| = 1
        % lambda_1, lambda_2
        b = sqrt(1.0 / t);
        a = s(i)*b;
        
        % q_yx = lambda_1*v1 + lambda_2*v2
        q_yx = (a*v1 + b*v2)'; % [w x y z]
        R_yxs(:,:,i) = quat2rotm(q_yx);
        eul(i, :) = rotm2eul(R_yxs(:,:,i));
    end
    if (abs(eul(1,1)) < abs(eul(2,1)))
        R_yx = R_yxs(:,:,1);
    else
        R_yx = R_yxs(:,:,2);
    end
    b_solve = true;
    
    sigma_threshold = 0.001;
    
    if sigma_2 > sigma_threshold
        b_converaged = true;
    else
        b_converaged = false;
    end
end



