function tau = Torque_control(q_d, q_d_dot, q_d_ddot, q, q_dot)

global M C M_tilde C_tilde G_tilde lambda e_q de_q acc vel e

K = 10*eye(2); % Positive definite diagonal matrix

tau = (M_tilde*acc)+ (C_tilde*vel) + (G_tilde) - K*e;

end