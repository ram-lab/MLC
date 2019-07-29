function [refine_Tform] = motion_base_refinement(Tform_base, Tform_ref, ini_Tform)
    refine_Tform = ini_Tform;

%     q = rotm2quat(ini_Tform(1:3, 1:3));
%     t = ini_Tform(:,1:3);
%    
%     % planar pose error
%     R1 = Tform_base(1:3, 1:3, j);
%     q1 = rotm2quat(R1);   
%     t1 = Tform_base(1:3, 1:3, j);
%     
%     R = quat2rotm(q);
%     
%     R2 = Tform_ref(1:3, 1:3, j);
%     q2 = rotm2quat(R2);
%     t2 = Tform_ref(1:3, 4, j);
%     t_error = (R1-eye(3,3)) * t - (R*t2) + t1;
%     
% %     q_error = quatmultiply(quatmultiply(quatconj(q), q1), quatmultiply(q, quatconj(q2)));
%     r = vrrotmat2vec(R1*R * (R*R2)^(-1));
%     r = r * r(4);
%     r_error = norm(r(1:3), 2); % |log(AX * (XB)^(-1))|, rad   


%     x = fmincon();
    
end








