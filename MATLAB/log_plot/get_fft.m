function [f, P1] = get_fft(raw_data, Ts)
    % get_fft computes the single-sided amplitude spectrum of a signal using FFT.
    % 
    % This function takes in a time-domain signal (`raw_data`) and its sampling time interval (`Ts`),
    % performs a Fast Fourier Transform (FFT), and returns the corresponding frequency vector (`f`)
    % and the single-sided amplitude spectrum (`P1`).
    %
    % Inputs:
    %   raw_data - Time-domain signal (1D array)
    %   Ts - Sampling time interval (scalar)
    %
    % Outputs:
    %   f - Frequency vector (1D array)
    %   P1 - Single-sided amplitude spectrum (1D array)
    L = length(raw_data);
    sampling_rate = 1/Ts;
    Raw_FFT = fft(raw_data);
    P2 = abs(Raw_FFT / L);
    P1 = P2(1:floor(L/2 + 1));
    P1(2:end - 1) = 2 * P1(2:end - 1);
    f = sampling_rate / L * (0:(L/2));
end