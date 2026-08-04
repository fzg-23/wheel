function [timeStamp, x_trues, hs] = extract_truestates_from_VICON(rawData, FPS, h_offset)
    r = 0.072; % wheel radius
    floor_z = -0.004;
    
    % 4행부터 데이터를 숫자 행렬로 변환
    data = cell2mat(rawData(4:end, [1,3:end]));
    data_len = size(data,1);


    % Frame to Time
    data(:,1) = data(:,1)/FPS;
    % Deg to Rad
    data(:,5:7) = deg2rad(data(:,5:7));
    
    % Extract datas
    timeStamp = data(:,1);
    pos_raw_vecs = data(:,2:4)';
    
    % Euler ZYX = roll-pitch-yaw
    rolls = data(:,5)';
    pitchs = data(:,6)';
    yaws = data(:,7)';
    
    R_WC = zeros(3, 3, data_len);
    R_CB = zeros(3, 3, data_len);
    R_WB = zeros(3, 3, data_len);
    
    p_W_woco_vecs = zeros(3, data_len);
    p_W_wobo_vecs = zeros(3, data_len);
    hs = zeros(data_len,1);
    for i = 1:data_len
        R_WC(:,:,i) = eulerZYXtoRotationMatrix(0,0,yaws(i));
        R_CB(:,:,i) = eulerZYXtoRotationMatrix(rolls(i),pitchs(i),0);
        R_WB(:,:,i) = R_WC(:,:,i) * R_CB(:,:,i);
        p_W_wobo_vecs(:,i) = pos_raw_vecs(:,i) + R_WB(:,:,i)*[0;0;h_offset] -[0;0;r] - [0;0;floor_z];
        hs(i) = p_W_wobo_vecs(3,i) / R_WB(3,3,i);
        p_W_woco_vecs(:,i) = p_W_wobo_vecs(:,i)-R_WB(:,:,i)*[0;0;hs(i)];
    end
    
    v_W_woco_vecs = gradient(p_W_woco_vecs) * FPS;
    v_C_woco_vecs = zeros(size(v_W_woco_vecs));
    for i = 1 : data_len
        v_C_woco_vecs(:,i) = R_WC(:,:,i)'*v_W_woco_vecs(:,i);
    end
    
    
    theta_trues = pitchs;
    theta_dot_trues = gradient(theta_trues) * FPS;
    v_trues = v_C_woco_vecs(1,:);
    psi_dot_trues = gradient(yaws) * FPS;
    
    x_trues = [theta_trues; theta_dot_trues; v_trues; psi_dot_trues];
end