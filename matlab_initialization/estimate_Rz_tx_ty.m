function [R_z, t_x, t_y] = estimate_Rz_tx_ty(Tform_base, Tform_ref, R_yx)
    G = zeros(2*length(Tform_base), 4);
    w = zeros(2*length(Tform_base), 1);
    for j = 1:length(Tform_base)
        R_base = Tform_base(1:2, 1:2, j);
        t_base = Tform_base(1:2, 4, j);
        t_ref = Tform_ref(1:3, 4, j);        
        
        J = R_base - eye(2,2);
        
        n = R_yx(3, :)';
        p = R_yx * (t_ref - dot(t_ref, n) * n);    
        K = [p(1), -p(2); p(2), p(1)];
        G((j-1)*2+1:j*2, :) = [J, K];
        w((j-1)*2+1:j*2, :) = -t_base;
    end
    m = G\w;
%     lambda = 0.005;
%     m = (G'*G+lambda*eye(4,4))^(-1)*G'*w; % regulator
    t_x = m(1);
    t_y = m(2);
    yaw = atan2(-m(4), -m(3));
    R_z = eul2rotm([yaw 0 0], 'ZYX');
end



