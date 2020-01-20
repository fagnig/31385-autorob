R = 2.0;
L = 0.0;
K_t = 0.004;
J = 0.5 * 10^(-4);
f = 0.00;
K_e = K_t;
P = 19 * K_e;
M_b = 0.008;
r = 500;

t_max = 10;
t = sim('motor_controller_sim')
v_max = max(v)

for i =1:length(t)
    if v(i) > 0.999*v_max
        i_max =i;
        break;
    end
end

T = t(1:i_max);
V = v(1:i_max);
V_max = v_max*ones(size(T));

plot(T,V, T,V_max)
legend({'variable velocity', 'limiting velocity'}, 'Location','southeast')
xlabel('time (seconds)')
ylabel('angular velocity (rad/s)')
grid on

