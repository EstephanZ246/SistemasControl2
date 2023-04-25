function [Kp,Ki,Kd] = SolverPID(Wn,Zeta,tr,ts,Mp,figuras)

% Armamos funcion de transferencia
syms s
s = tf('s')
G = Wn^2/(s^2 + 2*Zeta*Wn*s + Wn^2);
% Muestra como e ve el sistema sin control
if figuras == 1
figure(1)
step(G)
end
%{
tr, este se da en segundo
ts, este se da en segundos
Mp, este en %
%}

%Mínimo de valores para las constantes
Minimos = [0,0,0];
Maximos = [200,200,200];
options = optimoptions('fmincon', 'Display', 'iter');

fun = @(x) PID(x(1), x(2), x(3), G, tr, ts, Mp);

x0 = [1, 1, 1];
[x, fval] = fmincon(fun, x0, [], [], [], [], Minimos, Maximos, [], options);

% Mostrar resultados
Kp = x(1);
Ki = x(2);
Kd = x(3);
disp(['Kp = ' num2str(Kp)]);
disp(['Ki = ' num2str(Ki)]);
disp(['Kd = ' num2str(Kd)]);

% Se muestra el sistema controlado
if figuras == 1
C = pid(Kp, Ki, Kd);
T = feedback(C*G, 1);
figure(2)
step(T);
end

end

function Trabajo = PID(Kp, Ki, Kd, G, tr_max, ts_max, Mp_max)
    C = pid(Kp, Ki, Kd);
    T = feedback(C*G, 1);
    info = stepinfo(T);
    Trabajo = abs(info.RiseTime - tr_max) + abs(info.SettlingTime - ts_max) + abs(info.Overshoot - Mp_max);
end
