classdef EKF_c < handle
    properties
        x % State Vector
        z % Measurement Vector
        P % Covariance Matrix
        Q % Process Noise Covariance Matrix
        R % Sensor Noise Covariance Matrix

        F % Model Jacobian
        H % Observation model Jacobian
        x_dot
        h_obs

        dt % sampling time
        Pol % Calculate Model
    end

    methods
        function obj = EKF_c(initialState, initialCovariance, processNoise, measurementNoise, Pol_, Ts)
            obj.x = initialState;
            obj.P = initialCovariance;
            obj.Q = processNoise;
            obj.R = measurementNoise;
            obj.Pol = Pol_;
            obj.dt = Ts;

            % 상태 벡터 크기 확인
            stateSize = size(initialState, 1);
            measurementSize = size(measurementNoise, 1);

            obj.z = zeros(measurementSize, 1);


            % 예외 처리: Q의 크기 확인
            if ~isequal(size(processNoise), [stateSize, stateSize])
                error('Process noise Q must be a square matrix with size [%d, %d].', stateSize, stateSize);
            end

            % 예외 처리: P의 크기 확인
            if ~isequal(size(initialCovariance), [stateSize, stateSize])
                error('Initial covariance P must be a square matrix with size [%d, %d].', stateSize, stateSize);
            end
        end

        function obj = predict(obj, u, h)
            obj.Pol.setState(obj.x, u, h);
            obj.Pol.calculateDynamics();
            obj.Pol.calculateJacobian();

            [obj.x, obj.x_dot, obj.F] = obj.Pol.predict_state(obj.dt);
            obj.P = obj.F * obj.P * obj.F' + obj.Q; % 예측된 오차 공분산 행렬
        end

        function obj = update(obj, z)
            [obj.h_obs, obj.H] = obj.Pol.predict_measurement(obj.x_dot, obj.x);
            K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + obj.R); % 칼만 이득
            obj.x = obj.x + K * (z - obj.h_obs); % 상태 업데이트
            obj.P = (eye(length(obj.x)) - K * obj.H) * obj.P; % 오차 공분산 업데이트
        end

        function obj = update_test(obj, z, observe_model, Case_num)
            if Case_num == 1
                [obj.h_obs, obj.H] = observe_model.h_obs_f_case1(obj.x);
            elseif Case_num == 2
                [obj.h_obs, obj.H] = observe_model.h_obs_f_case2(obj.x, obj.Pol.h);
            elseif Case_num == 3
                [obj.h_obs, obj.H] = observe_model.h_obs_f_case3(obj.x);
            end
            K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + obj.R); % 칼만 이득
            obj.x = obj.x + K * (z - obj.h_obs); % 상태 업데이트
            obj.P = (eye(length(obj.x)) - K * obj.H) * obj.P; % 오차 공분산 업데이트
        end
    end
end
