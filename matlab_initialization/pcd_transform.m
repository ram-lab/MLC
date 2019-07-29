function pcd_transform(file_pcd, file_trans_pcd)
    pc = pcread(file_pcd);
    x = pc.Location(:,1);
    y = pc.Location(:,2);
    z = pc.Location(:,3);
    xyz_points = [z, x, y];
    if ~isempty(pc.Intensity)
        i = pc.Intensity;
        pc_trans = pointCloud(xyz_points, 'Intensity', i); 
    else
        pc_trans = pointCloud(xyz_points); 
    end
    pcwrite(pc_trans, file_trans_pcd);
end