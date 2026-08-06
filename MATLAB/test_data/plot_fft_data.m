function [outputArg1,outputArg2] = plot_fft_data(filename1, filename2)
    if ~endsWith(filename1, '.csv', 'IgnoreCase', true)
        error('InputError:NotCSV', 'The provided file "%s" is not a CSV file.', filename1);
    end
    if ~isfile(filename1)
        error('FileError:NotFound', 'The file "%s" does not exist.', filename1);
    end
    if ~endsWith(filename2, '.csv', 'IgnoreCase', true)
        error('InputError:NotCSV', 'The provided file "%s" is not a CSV file.', filename2);
    end
    if ~isfile(filename2)
        error('FileError:NotFound', 'The file "%s" does not exist.', filename2);
    end

    rawData1 = readmatrix(filename1);
    rawData2 = readmatrix(filename2);

    timeStamp1 = rawData1(:, 1)/1000;     % 第一列：时间戳
    acc_x1 = rawData1(:, 2);              % 第 2 列：acc_x
    acc_y1 = rawData1(:, 3);              % 第 3 列：acc_y
    acc_z1 = rawData1(:, 4);              % 第 4 列：acc_z
    gyr_x1 = rawData1(:, 5);              % 第 5 列：gyr_x
    gyr_y1 = rawData1(:, 6);              % 第 6 列：gyr_y
    gyr_z1 = rawData1(:, 7);              % 第 7 列：gyr_z
    h_d1 = rawData1(:, 8);                % 第 8 列：h_d
    psi_dot_d1 = rawData1(:, 9);          % 第 9 列：psi_dot_d
    psi_dot_hat1 = rawData1(:, 10);       % 第 10 列：psi_dot_hat
    tau_LW1 = rawData1(:, 11);            % 第 11 栏：tau_LW
    tau_RW1 = rawData1(:, 12);            % 第 12 列：tau_RW
    theta_dot_hat1 = rawData1(:, 15);     % 第 15 栏：theta_dot_hat
    theta_eq1 = rawData1(:, 16);          % 第 16 列：theta_eq
    theta_hat1 = rawData1(:, 17);         % 第 17 栏：theta_hat
    v_d1 = rawData1(:, 18);               %第 18 栏：v_d
    v_hat1 = rawData1(:, 19);             % 第 19 栏：v_hat
    
    timeStamp2 = rawData2(:, 1)/1000;     % 第一列：时间戳
    acc_x2 = rawData2(:, 2);              % 第 2 列：acc_x
    acc_y2 = rawData2(:, 3);              % 第 3 列：acc_y
    acc_z2 = rawData2(:, 4);              % 第 4 列：acc_z
    gyr_x2 = rawData2(:, 5);              % 第 5 列：gyr_x
    gyr_y2 = rawData2(:, 6);              % 第 6 列：gyr_y
    gyr_z2 = rawData2(:, 7);              % 第 7 列：gyr_z
    h_d2 = rawData2(:, 8);                % 第 8 列：h_d
    psi_dot_d2 = rawData2(:, 9);          % 第 9 列：psi_dot_d
    psi_dot_hat2 = rawData2(:, 10);       % 第 10 列：psi_dot_hat
    tau_LW2 = rawData2(:, 11);            % 第 11 栏：tau_LW
    tau_RW2 = rawData2(:, 12);            % 第 12 列：tau_RW
    theta_dot_hat2 = rawData2(:, 15);     % 第 15 栏：theta_dot_hat
    theta_eq2 = rawData2(:, 16);          % 第 16 列：theta_eq
    theta_hat2 = rawData2(:, 17);         % 第 17 栏：theta_hat
    v_d2 = rawData2(:, 18);               %第 18 栏：v_d
    v_hat2 = rawData2(:, 19);             % 第 19 栏：v_hat
    
    acc_x_max = get_max_val(acc_x1, acc_x2);
    acc_y_max = get_max_val(acc_y1, acc_y2);
    acc_z_max = get_max_val(acc_z1, acc_z2);

    figure(1);
    subplot(1, 3, 1);
    [f, P1] = get_fft(acc_x1);
    plot(f(2:end), P1(2:end));
    xlabel("frequency domain");
    ylabel("Scaled acc\_x");
    title("Raw Data FFT");
    ylim([0, acc_x_max]);
    grid on;
    
    subplot(1, 3, 2);
    [f, P1] = get_fft(acc_y1);
    plot(f(2:end), P1(2:end));
    xlabel("frequency domain");
    ylabel("Scaled acc\_y");
    title("Raw Data FFT");
    ylim([0, acc_y_max]);
    grid on;
    
    subplot(1, 3, 3);
    [f, P1] = get_fft(acc_z1);
    plot(f(2:end), P1(2:end));
    xlabel("frequency domain");
    ylabel("Scaled acc\_z");
    title("Raw Data FFT");
    ylim([0, acc_z_max]);
    grid on;
    
    figure(2);
    subplot(1, 3, 1);
    [f, P1] = get_fft(acc_x2);
    plot(f(2:end), P1(2:end));
    xlabel("frequency domain");
    ylabel("Scaled acc\_x");
    title("Raw Data FFT");
    ylim([0, acc_x_max]);
    grid on;
    
    subplot(1, 3, 2);
    [f, P1] = get_fft(acc_y2);
    plot(f(2:end), P1(2:end));
    xlabel("frequency domain");
    ylabel("Scaled acc\_y");
    title("Raw Data FFT");
    ylim([0, acc_y_max]);
    grid on;
    
    subplot(1, 3, 3);
    [f, P1] = get_fft(acc_z2);
    plot(f(2:end), P1(2:end));
    xlabel("frequency domain");
    ylabel("Scaled acc\_z");
    title("Raw Data FFT");
    ylim([0, acc_z_max]);
    grid on;
end

function max_val = get_max_val(data1, data2)
    [f, P1] = get_fft(data1);
    [f, P2] = get_fft(data2);
    max_val = max([P1(2:end); P2(2:end)]);
end

