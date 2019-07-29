function [matrix_tform] = SE3_to_matrix(SE3_tform)
    matrix_tform = [[SE3_tform.n;0], [SE3_tform.o;0], [SE3_tform.a;0], [SE3_tform.t;1]];
end