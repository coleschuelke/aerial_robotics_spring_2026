clear variables;
close all;
clc;


run("../Params/quadParamsScript.m")

% Transfer Function 
numerator = [quadParams.cm(1)];
denom = [quadParams.taum(1), 1];

motor = tf(numerator, denom);
[y, t] = step(motor);
stepinfo(motor)

% ODE 
f = @(t, omega) domegadt(omega, 1, quadParams);
[T, x] = ode45(f, [0, 0.3], 0);

figure;
plot(T, x, 'LineWidth', 3)
set(gca, 'FontSize', 14);
xlabel("Time (seconds)", 'FontSize', 20)
ylabel("Amplitude", 'FontSize', 20)
title('Step Response ODE', 'FontSize', 20)

figure;
plot(t, y, 'LineWidth', 3)
set(gca, 'FontSize', 14);
xlabel("Time (seconds)", 'FontSize', 20)
ylabel("Amplitude", 'FontSize', 20)
title('Step Response TF', 'FontSize', 20)

function[omegadot] = domegadt(omega, ea, qp)
    omegadot = (qp.cm(1)*ea - omega) / qp.taum(1);
end
