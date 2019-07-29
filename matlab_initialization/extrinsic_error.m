function [rotation_error, translation_error] = extrinsic_error(A, B)
    % miccalibration detection 2, stable
    R_delta = A(1:3,1:3) * B(1:3,1:3)^(-1);
    r = vrrotmat2vec(R_delta);
    r = r * r(4);
    rotation_error = norm(r(1:3), 2); % |log(AX * (XB)^(-1))^{V}|, rad

    t_error = A(1:3, 4) - B(1:3, 4); % |tA - tB|
    translation_error = norm(t_error, 2); % use the translation_error to detect miscalibration    
end