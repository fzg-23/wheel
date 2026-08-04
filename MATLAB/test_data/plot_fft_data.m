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

    timeStamp1 = rawData1(:, 1)/1000;     % 1열: TimeStamp
    acc_x1 = rawData1(:, 2);              % 2열: acc_x
    acc_y1 = rawData1(:, 3);              % 3열: acc_y
    acc_z1 = rawData1(:, 4);              % 4열: acc_z
    gyr_x1 = rawData1(:, 5);              % 5열: gyr_x
    gyr_y1 = rawData1(:, 6);              % 6열: gyr_y
    gyr_z1 = rawData1(:, 7);              % 7열: gyr_z
    h_d1 = rawData1(:, 8);                % 8열: h_d
    psi_dot_d1 = rawData1(:, 9);          % 9열: psi_dot_d
    psi_dot_hat1 = rawData1(:, 10);       % 10열: psi_dot_hat
    tau_LW1 = rawData1(:, 11);            % 11열: tau_LW
    tau_RW1 = rawData1(:, 12);            % 12열: tau_RW
    theta_dot_hat1 = rawData1(:, 15);     % 15열: theta_dot_hat
    theta_eq1 = rawData1(:, 16);          % 16열: theta_eq
    theta_hat1 = rawData1(:, 17);         % 17열: theta_hat
    v_d1 = rawData1(:, 18);               % 18열: v_d
    v_hat1 = rawData1(:, 19);             % 19열: v_hat
    
    timeStamp2 = rawData2(:, 1)/1000;     % 1열: TimeStamp
    acc_x2 = rawData2(:, 2);              % 2열: acc_x
    acc_y2 = rawData2(:, 3);              % 3열: acc_y
    acc_z2 = rawData2(:, 4);              % 4열: acc_z
    gyr_x2 = rawData2(:, 5);              % 5열: gyr_x
    gyr_y2 = rawData2(:, 6);              % 6열: gyr_y
    gyr_z2 = rawData2(:, 7);              % 7열: gyr_z
    h_d2 = rawData2(:, 8);                % 8열: h_d
    psi_dot_d2 = rawData2(:, 9);          % 9열: psi_dot_d
    psi_dot_hat2 = rawData2(:, 10);       % 10열: psi_dot_hat
    tau_LW2 = rawData2(:, 11);            % 11열: tau_LW
    tau_RW2 = rawData2(:, 12);            % 12열: tau_RW
    theta_dot_hat2 = rawData2(:, 15);     % 15열: theta_dot_hat
    theta_eq2 = rawData2(:, 16);          % 16열: theta_eq
    theta_hat2 = rawData2(:, 17);         % 17열: theta_hat
    v_d2 = rawData2(:, 18);               % 18열: v_d
    v_hat2 = rawData2(:, 19);             % 19열: v_hat
    
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

