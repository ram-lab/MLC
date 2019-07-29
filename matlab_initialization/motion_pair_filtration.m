function [theta_error, d_error] = motion_pair_filtration(A, B)
    % check the invariance of geometric properties based on screw motion
    % theta_error and d_error
    % should be equal to zero
    
    r_A = vrrotmat2vec(A(1:3,1:3));
    r_B = vrrotmat2vec(B(1:3,1:3));
    theta_A = r_A(4);
    theta_B = r_B(4);
    r_A = r_A(1:3);
    r_B = r_B(1:3);    
    theta_error = abs(theta_A-theta_B);
    d_A = dot(A(1:3,4), r_A);
    d_B = dot(B(1:3,4), r_B);
    d_error = abs(d_A-d_B);
end