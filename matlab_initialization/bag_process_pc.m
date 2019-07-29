function [pc] = bag_process_pc(bag_file, topic)
    bag_select = rosbag(bag_file);
    topic_select = select(bag_select, ...
        'Topic', topic);
    msg_pc = readMessages(topic_select, 'DataFormat', 'struct');
    pc = cell(length(msg_pc));
    for i = 1:length(msg_pc)
        
%         pc{i} = pointCloud(readXYZ(msg_pc));
    end
end