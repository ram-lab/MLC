function [rotation_error, translation_error] = miscalibration_error(A, B, X, c)
    R = X(1:3, 1:3);
    t = X(1:3, 4);
    if strcmp(c, 'vector')
        % miccalibration detection 1, unstable
        r_A = vrrotmat2vec(A(1:3,1:3));
        r_B = vrrotmat2vec(B(1:3,1:3));
        if abs(r_A(4)) < 0.0005
            r_A = zeros(3,1);
        else
            r_A = r_A(1:3)';
        end
        if abs(r_B(4)) < 0.0005
            r_B = zeros(3,1);
        else
            r_B = r_B(1:3)';
        end     
        r_error = r_A - R*r_B;
        rotation_error = norm(r_error, 2); % |RrB-rA|, length

        t_error = (A(1:3,1:3) - eye(3))*t + A(1:3,4) - R*B(1:3,4); % |(RA-I)*t+tA-RtB|
        translation_error = norm(t_error, 2); % use the translation_error to detect miscalibration
        
    elseif strcmp(c, 'rad')
        % miccalibration detection 2, stable
        r = vrrotmat2vec(A(1:3,1:3)*X(1:3,1:3) * (X(1:3,1:3)*B(1:3,1:3))^(-1));
%         r = r * r(4);
%         rotation_error = norm(r(1:3), 2); % |log(AX * (XB)^(-1))|, rad
        rotation_error = abs(r(4));

        t_error = (A(1:3,1:3) - eye(3))*t + A(1:3,4) - R*B(1:3,4); % |(RA-I)*t+tA-RtB|
        translation_error = norm(t_error, 2); % use the translation_error to detect miscalibration    
    end
end