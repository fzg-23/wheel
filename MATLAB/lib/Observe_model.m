classdef Observe_model < handle
    properties
        params
        L
        R
        g
        h
        phi
        p_bcom
    end

    methods
        function obj = Observe_model(parameters)
            obj.params = parameters;
            obj.L = obj.params.L;
            obj.R = obj.params.R;
            obj.g = obj.params.g;
        end

        function [h_obs, H] = h_obs_f_case1(obj, x)
            % observation model
            % x : state
            theta = x(1);
            theta_dot = x(2);
            v = x(3);
            psi_dot = x(4);

            cos_theta = cos(theta);
            sin_theta = sin(theta);

            % h_obs计算
            h_obs = zeros(5, 1); % 用于存储结果的向量
            h_obs(1) = - obj.g * sin_theta ;
            h_obs(2) = obj.g * cos_theta ;
            h_obs(3) = theta_dot;
            h_obs(4) = theta_dot - v / obj.R - (obj.L * psi_dot) / obj.R;
            h_obs(5) = -theta_dot + v / obj.R - (obj.L * psi_dot) / obj.R;

            % H矩阵计算
            H = zeros(5, 4); % 8x4矩阵
            H(1,:) = [- obj.g * cos_theta, 0, 0, 0];
            H(2,:) = [- obj.g * sin_theta, 0, 0, 0];
            H(3,:) = [0, 1, 0, 0];
            H(4,:) = [0, 1, -1/obj.R, -obj.L/obj.R];
            H(5,:) = [0, -1, 1/obj.R, -obj.L/obj.R];
        end

        function [h_obs, H] = h_obs_f_case2(obj, x, h)
            obj.h = h;
            
            % observation model
            % x : state
            theta = x(1);
            theta_dot = x(2);
            v = x(3);
            psi_dot = x(4);

            cos_theta = cos(theta);
            cos_2theta = cos(2*theta);
            sin_theta = sin(theta);
            sin_2theta = sin(2*theta);
            
                
            % h_obs计算
            h_obs = zeros(5, 1); % 用于存储结果的向量
            h_obs(1) = - (obj.h / 2) * sin_2theta * psi_dot^2 - obj.g * sin_theta ;
            h_obs(2) = -obj.h * theta_dot^2 - (obj.h / 2) * (1- cos_2theta) * psi_dot^2  + obj.g * cos_theta ;
            h_obs(3) = theta_dot;
            h_obs(4) = theta_dot - v / obj.R - (obj.L * psi_dot) / obj.R;
            h_obs(5) = -theta_dot + v / obj.R - (obj.L * psi_dot) / obj.R;

            % H矩阵计算
            H = zeros(5, 4); % 8x4矩阵
            H(1,:) = [-obj.h*cos_2theta*psi_dot^2 - obj.g * cos_theta, 0, 0, -obj.h*sin_2theta*psi_dot];
            H(2,:) = [-obj.h*sin_2theta*psi_dot^2 - obj.g * sin_theta, -2*obj.h*theta_dot, 0, -obj.h*(1-cos_2theta)*psi_dot];
            H(3,:) = [0, 1, 0, 0];
            H(4,:) = [0, 1, -1/obj.R, -obj.L/obj.R];
            H(5,:) = [0, -1, 1/obj.R, -obj.L/obj.R];
        end

        function [h_obs, H] = h_obs_f_case3(obj, x)
            % observation model
            % x : state
            theta = x(1);
            theta_dot = x(2);
            v = x(3);
            psi_dot = x(4);

            % h_obs计算
            h_obs = zeros(4, 1); % 用于存储结果的向量
            h_obs(1) = theta ;
            h_obs(2) = theta_dot ;
            h_obs(3) = theta_dot - v / obj.R - (obj.L * psi_dot) / obj.R;
            h_obs(4) = -theta_dot - v / obj.R - (obj.L * psi_dot) / obj.R;
            
            % H矩阵计算
            H = zeros(4, 4); % 8x4矩阵
            H(1,:) = [1, 0, 0, 0];
            H(2,:) = [0, 1, 0, 0];
            H(3,:) = [0, 1, -1/obj.R, -obj.L/obj.R];
            H(4,:) = [0, -1, 1/obj.R, -obj.L/obj.R];
        end

    end
end
