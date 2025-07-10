function [tfz] = EncontraTFz(rpm,fatorSetPoint,t,Ts)
J = 10; % Janela do filtro média móvel
rpm = movmean(rpm,J);

% Realizando média de 3 segundos até 5 segundos
Media = mean(rpm(250:500));

K = Media/fatorSetPoint;

[idx] = closest(rpm,Media*0.632);

Tau = t(idx);

tfa = tf(K, [Tau 1]);

tfz = c2d(tfa,Ts,'zoh');

end

