function v = tf_to_vector(tform)
    v = zeros(1, 7);
    v(1) = tform(1, 4);
    v(2) = tform(2, 4);
    v(3) = tform(3, 4);
    v(4:7) = rotm2quat(tform(1:3, 1:3));
end