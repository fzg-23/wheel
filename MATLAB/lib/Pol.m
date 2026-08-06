classdef Pol < handle
    properties
        x % State Vector
        u % Input Vector

        params
        I_B_B
        I_B_RW
        I_B_LW
        m_B
        m_RW
        m_LW
        L
        R
        g
        h
        phi
        p_bcom

        func

        % Dynamic Model Matrices
        M
        M_inv
        nle
        B

        % Jacobian Matrices
        dM_dtheta
        dnle_dtheta
        dnle_dqdot
    end

    methods
        function obj = Pol(parameters,functions)
            obj.params = parameters;
            obj.func = functions;
            obj.L = obj.params.L;
            obj.R = obj.params.R;
            obj.g = obj.params.g;

            obj.m_RW = obj.params.m_RW;
            obj.m_LW = obj.params.m_LW;
            obj.I_B_RW = obj.params.I_B_RW;
            obj.I_B_LW = obj.params.I_B_LW;
        end

        function obj = setState(obj, state, input, height)
            obj.x = state;
            obj.u = input;
            obj.h = height;
            obj.phi = 0;
            [obj.m_B, obj.p_bcom, obj.I_B_B] = calculate_body_total_property(obj.h, obj.phi, obj.params);
        end

        function theta_eq = get_theta_eq(obj, h, phi)
            [~, p_bcom_, ~] = calculate_body_total_property(h, phi, obj.params);
            theta_eq = atan(-p_bcom_(1) / (h + p_bcom_(3)));
        end

        function obj = calculateDynamics(obj)
            obj.calculate_M();
            obj.calculate_M_inv();
            obj.calculate_nle();
            obj.calculate_B();
        end

        function obj = calculateJacobian(obj)
            obj.calculate_dM_dtheta();
            obj.calculate_dnle_dtheta();
            obj.calculate_dnle_dqdot();
        end

        function obj = calculate_M(obj)
            obj.M = obj.func.M_f(obj.I_B_B(1, 1), obj.I_B_B(2, 1), obj.I_B_B(2, 2), ...
                obj.I_B_B(3, 1), obj.I_B_B(3, 2), obj.I_B_B(3, 3), ...
                obj.I_B_LW(1, 1), obj.I_B_RW(1, 1), obj.I_B_LW(2, 1), ...
                obj.I_B_LW(2, 2), obj.I_B_RW(2, 1), obj.I_B_RW(2, 2), ...
                obj.I_B_LW(3, 1), obj.I_B_LW(3, 2), obj.I_B_LW(3, 3), ...
                obj.I_B_RW(3, 1), obj.I_B_RW(3, 2), obj.I_B_RW(3, 3), ...
                obj.L, obj.R, obj.h, obj.m_B, obj.m_LW, obj.m_RW, ...
                obj.p_bcom(1), obj.p_bcom(2), obj.p_bcom(3), obj.x(1));
        end

        function obj = calculate_M_inv(obj)
            obj.M_inv = obj.M \ eye(size(obj.M));
        end

        function obj = calculate_nle(obj)
            obj.nle = obj.func.nle_f(obj.I_B_B(1, 1), obj.I_B_B(2, 1), obj.I_B_B(3, 1), ...
                obj.I_B_B(3, 2), obj.I_B_B(3, 3), obj.I_B_LW(1, 1), ...
                obj.I_B_RW(1, 1), obj.I_B_LW(2, 1), obj.I_B_RW(2, 1), ...
                obj.I_B_LW(3, 1), obj.I_B_LW(3, 2), obj.I_B_LW(3, 3), ...
                obj.I_B_RW(3, 1), obj.I_B_RW(3, 2), obj.I_B_RW(3, 3), ...
                obj.L, obj.R, obj.g, obj.h, obj.m_B, ...
                obj.p_bcom(1), obj.p_bcom(2), obj.p_bcom(3), ...
                obj.x(4), obj.x(1), obj.x(2), obj.x(3));
        end

        function obj = calculate_B(obj)
            obj.B = obj.func.B_f(obj.L, obj.R);
        end

        function obj = calculate_dM_dtheta(obj)
            obj.dM_dtheta = obj.func.dM_dtheta_f(obj.I_B_B(1, 1), obj.I_B_B(2, 1), obj.I_B_B(3, 1), ...
                obj.I_B_B(3, 2), obj.I_B_B(3, 3), obj.I_B_LW(1, 1), ...
                obj.I_B_RW(1, 1), obj.I_B_LW(2, 1), obj.I_B_RW(2, 1), ...
                obj.I_B_LW(3, 1), obj.I_B_LW(3, 2), obj.I_B_LW(3, 3), ...
                obj.I_B_RW(3, 1), obj.I_B_RW(3, 2), obj.I_B_RW(3, 3), ...
                obj.L, obj.R, obj.h, obj.m_B, obj.p_bcom(1), ...
                obj.p_bcom(2), obj.p_bcom(3), obj.x(1));
        end

        function obj = calculate_dnle_dtheta(obj)
            obj.dnle_dtheta = obj.func.dnle_dtheta_f(obj.I_B_B(1, 1), obj.I_B_B(2, 1), obj.I_B_B(3, 1), ...
                obj.I_B_B(3, 2), obj.I_B_B(3, 3), obj.I_B_LW(1, 1), ...
                obj.I_B_RW(1, 1), obj.I_B_LW(2, 1), obj.I_B_RW(2, 1), ...
                obj.I_B_LW(3, 1), obj.I_B_LW(3, 2), obj.I_B_LW(3, 3), ...
                obj.I_B_RW(3, 1), obj.I_B_RW(3, 2), obj.I_B_RW(3, 3), ...
                obj.L, obj.R, obj.g, obj.h, obj.m_B, ...
                obj.p_bcom(1), obj.p_bcom(2), obj.p_bcom(3), ...
                obj.x(4), obj.x(1), obj.x(2), obj.x(3));
        end

        function obj = calculate_dnle_dqdot(obj)
            obj.dnle_dqdot = obj.func.dnle_dqdot_f(obj.I_B_B(1, 1), obj.I_B_B(2, 1), obj.I_B_B(3, 1), ...
                obj.I_B_B(3, 2), obj.I_B_B(3, 3), obj.I_B_LW(1, 1), ...
                obj.I_B_RW(1, 1), obj.I_B_LW(2, 1), obj.I_B_RW(2, 1), ...
                obj.I_B_LW(3, 1), obj.I_B_LW(3, 2), obj.I_B_LW(3, 3), ...
                obj.I_B_RW(3, 1), obj.I_B_RW(3, 2), obj.I_B_RW(3, 3), ...
                obj.L, obj.R, obj.h, obj.m_B, obj.p_bcom(1), ...
                obj.p_bcom(2), obj.p_bcom(3), obj.x(4), ...
                obj.x(1), obj.x(2), obj.x(3));
        end

        function [x_pred, x_dot, F] = predict_state(obj, dt)
            x_dot = [obj.x(2); obj.M_inv * (-obj.nle + obj.B * obj.u)];
            x_pred = obj.x + x_dot * dt;

            F = zeros(4,4);
            F(1,2) = 1;
            F(2:4,1) = obj.M_inv * (obj.dM_dtheta * obj.M_inv * (obj.nle - obj.B * obj.u) - obj.dnle_dtheta);
            F(2:4, 2:4) = -obj.M_inv*obj.dnle_dqdot;
        end

        function [h_obs, H] = predict_measurement(obj, x_dot, x_pred)
            % theta_ddot = x_dot(2);
            % v_dot = x_dot(3);
            % psi_ddot = x_dot(4);

            theta_ddot = 0;
            v_dot = 0;
            psi_ddot = 0;


            theta = x_pred(1);
            theta_dot = x_pred(2);
            v = x_pred(3);
            psi_dot = x_pred(4);

            cos_theta = cos(theta);
            sin_theta = sin(theta);

            % h_obs计算
            h_obs = zeros(8, 1); % 用于存储结果的向量
            h_obs(1) = obj.h * theta_ddot + v_dot * cos_theta - obj.g * sin_theta - obj.h * psi_dot^2 * cos_theta * sin_theta;
            h_obs(2) = psi_dot * v + obj.h * psi_ddot * sin_theta + obj.h * psi_dot * theta_dot * cos_theta * 2.0;
            h_obs(3) = -obj.h * theta_dot^2 + obj.g * cos_theta + v_dot * sin_theta - psi_dot^2 * (obj.h - obj.h * cos_theta^2);
            % h_obs(1) = - obj.g * sin_theta;
            % h_obs(2) = 0;
            % h_obs(3) =  obj.g * cos_theta ;
            h_obs(4) = -psi_dot * sin_theta;
            h_obs(5) = theta_dot;
            h_obs(6) = psi_dot * cos_theta;
            h_obs(7) = theta_dot - v / obj.R - (obj.L * psi_dot) / obj.R;
            h_obs(8) = -theta_dot + v / obj.R - (obj.L * psi_dot) / obj.R;

            % H矩阵计算
            H = zeros(8, 4); % 8x4矩阵
            H(1, 1) = psi_dot^2 * (obj.h * sin_theta^2 - obj.h * cos_theta^2) - obj.g * cos_theta - v_dot * sin_theta;
            H(2, 1) = obj.h * psi_ddot * cos_theta - obj.h * psi_dot * theta_dot * sin_theta * 2.0;
            H(3, 1) = v_dot * cos_theta - obj.g * sin_theta - obj.h * psi_dot^2 * cos_theta * sin_theta * 2.0;
            H(4, 1) = -psi_dot * cos_theta;
            H(5, 1) = 0.0;
            H(6, 1) = -psi_dot * sin_theta;
            H(7, 1) = 0.0;
            H(8, 1) = 0.0;

            H(1, 2) = 0.0;
            H(2, 2) = obj.h * psi_dot * cos_theta * 2.0;
            H(3, 2) = obj.h * theta_dot * -2.0;
            H(4, 2) = 0.0;
            H(5, 2) = 1.0;
            H(6, 2) = 0.0;
            H(7, 2) = 1.0;
            H(8, 2) = -1.0;

            H(1, 3) = 0.0;
            H(2, 3) = psi_dot;
            H(3, 3) = 0.0;
            H(4, 3) = 0.0;
            H(5, 3) = 0.0;
            H(6, 3) = 0.0;
            H(7, 3) = -1.0 / obj.R;
            H(8, 3) = 1.0 / obj.R;

            H(1, 4) = obj.h * psi_dot * cos_theta * sin_theta * -2.0;
            H(2, 4) = v + obj.h * theta_dot * cos_theta * 2.0;
            H(3, 4) = psi_dot * (obj.h - obj.h * cos_theta^2) * -2.0;
            H(4, 4) = -sin_theta;
            H(5, 4) = 0.0;
            H(6, 4) = cos_theta;
            H(7, 4) = -obj.L / obj.R;
            H(8, 4) = -obj.L / obj.R;
        end

        function h_obs = get_measurement_truth(obj)

            x_dot = [obj.x(2); obj.M_inv * (-obj.nle + obj.B * obj.u)];

            theta_ddot = x_dot(2);
            v_dot = x_dot(3);
            psi_ddot = x_dot(4);

            % theta_ddot = 0;
            % v_dot = 0;
            % psi_ddot = 0;


            theta = obj.x(1);
            theta_dot = obj.x(2);
            v = obj.x(3);
            psi_dot = obj.x(4);

            cos_theta = cos(theta);
            sin_theta = sin(theta);

            % h_obs计算
            h_obs = zeros(8, 1); % 用于存储结果的向量
            h_obs(1) = obj.h * theta_ddot + v_dot * cos_theta - obj.g * sin_theta - obj.h * psi_dot^2 * cos_theta * sin_theta;
            h_obs(2) = psi_dot * v + obj.h * psi_ddot * sin_theta + obj.h * psi_dot * theta_dot * cos_theta * 2.0;
            h_obs(3) = -obj.h * theta_dot^2 + obj.g * cos_theta + v_dot * sin_theta - psi_dot^2 * (obj.h - obj.h * cos_theta^2);
            h_obs(4) = -psi_dot * sin_theta;
            h_obs(5) = theta_dot;
            h_obs(6) = psi_dot * cos_theta;
            h_obs(7) = theta_dot - v / obj.R - (obj.L * psi_dot) / obj.R;
            h_obs(8) = -theta_dot + v / obj.R - (obj.L * psi_dot) / obj.R;
        end
    end
end
