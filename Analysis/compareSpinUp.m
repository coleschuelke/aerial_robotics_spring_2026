clear variables;
close all;
clc;


run("../Params/quadParamsScript.m")

% Transfer Function 
numerator = [quadParams.cm(1)];
denom = [quadParams.taum(1), 1];

motor = tf(numerator, denom);
step(motor)
stepinfo(motor)

% ODE 
f = @(t, omega) domegadt(omega, 1, quadParams);
[T, x] = ode45(f, [0, 0.3], 0);

figure;
plot(T, x)
xlabel("Time (seconds)")
ylabel("Amplitude")
title('Step Response')

function[omegadot] = domegadt(omega, ea, qp)
    omegadot = (qp.cm(1)*ea - omega) / qp.taum(1);
end
