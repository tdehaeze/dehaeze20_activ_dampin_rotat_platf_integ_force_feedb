%% Clear Workspace and Close figures
clear; close all; clc;

%% Intialize Laplace variable
s = zpk('s');

% Numerical Values
% Let's define initial values for the model.

k = 1;    % Actuator Stiffness [N/m]
c = 0.05; % Actuator Damping [N/(m/s)]
m = 1;    % Payload mass [kg]

xi = c/(2*sqrt(k*m));
w0 = sqrt(k/m); % [rad/s]

% Campbell Diagram
% The Campbell Diagram displays the evolution of the real and imaginary parts of the system as a function of the rotating speed.

% It is shown in Figure [[fig:campbell_diagram]], and one can see that the system becomes unstable for $\Omega > \omega_0$ (the real part of one of the poles becomes positive).


Ws = linspace(0, 2, 51); % Vector of considered rotation speed [rad/s]

p_ws = zeros(4, length(Ws));

for W_i = 1:length(Ws)
    W = Ws(W_i);

    pole_G = pole(1/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2));
    [~, i_sort] = sort(imag(pole_G));
    p_ws(:, W_i) = pole_G(i_sort);
end

clear pole_G;

figure;

ax1 = subplot(1,2,1);
hold on;
for p_i = 1:size(p_ws, 1)
    plot(Ws, real(p_ws(p_i, :)), 'k-')
end
plot(Ws, zeros(size(Ws)), 'k--')
hold off;
xlabel('Rotational Speed [rad/s]'); ylabel('Real Part');

ax2 = subplot(1,2,2);
hold on;
for p_i = 1:size(p_ws, 1)
    plot(Ws, imag(p_ws(p_i, :)), 'k-')
end
hold off;
xlabel('Rotational Speed [rad/s]'); ylabel('Imaginary Part');

% Simscape Model
% In order to validate all the equations of motion, a Simscape model of the same system has been developed.
% The dynamics of the system can be identified from the Simscape model and compare with the analytical model.

% The rotating speed for the Simscape Model is defined.

W = 0.1; % Rotation Speed [rad/s]

Kiff = tf(zeros(2));

kp = 0; % Parallel Stiffness [N/m]
cp = 0; % Parallel Damping [N/(m/s)]

open('rotating_frame.slx');



% The transfer function from $[F_u, F_v]$ to $[d_u, d_v]$ is identified from the Simscape model.


%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/K'], 1, 'openinput');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/G'], 2, 'openoutput'); io_i = io_i + 1;

G = linearize(mdl, io, 0);

%% Input/Output definition
G.InputName  = {'Fu', 'Fv'};
G.OutputName = {'du', 'dv'};



% The same transfer function from $[F_u, F_v]$ to $[d_u, d_v]$ is written down from the analytical model.

Gth = (1/k)/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
      [(s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2), 2*W*s/(w0^2) ; ...
       -2*W*s/(w0^2), (s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)];



% Both transfer functions are compared in Figure [[fig:plant_simscape_analytical]] and are found to perfectly match.


freqs = logspace(-1, 1, 1000);

figure;
ax1 = subplot(2, 2, 1);
hold on;
plot(freqs, abs(squeeze(freqresp(G(1,1), freqs))), '-')
plot(freqs, abs(squeeze(freqresp(Gth(1,1), freqs))), '--')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [m/N]');
title('$d_u/F_u$, $d_v/F_v$');

ax3 = subplot(2, 2, 3);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(G(1,1), freqs))), '-')
plot(freqs, 180/pi*angle(squeeze(freqresp(Gth(1,1), freqs))), '--')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;

ax2 = subplot(2, 2, 2);
hold on;
plot(freqs, abs(squeeze(freqresp(G(1,2), freqs))), '-')
plot(freqs, abs(squeeze(freqresp(Gth(1,2), freqs))), '--')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [m/N]');
title('$d_u/F_v$, $d_v/F_u$');

ax4 = subplot(2, 2, 4);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(G(1,2), freqs))), '-', ...
     'DisplayName', 'Simscape')
plot(freqs, 180/pi*angle(squeeze(freqresp(Gth(1,2), freqs))), '--', ...
     'DisplayName', 'Analytical')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;
legend('location', 'southwest');

linkaxes([ax1,ax2,ax3,ax4],'x');
xlim([freqs(1), freqs(end)]);
linkaxes([ax1,ax2],'y');

% Effect of the rotation speed
% The transfer functions from $[F_u, F_v]$ to $[d_u, d_v]$ are identified for the following rotating speeds.

Ws = [0, 0.2, 0.7, 1.1]*w0; % Rotating Speeds [rad/s]

Gs = {zeros(2, 2, length(Ws))};

for W_i = 1:length(Ws)
    W = Ws(W_i);

    Gs(:, :, W_i) = {(1/k)/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
                     [(s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2), 2*W*s/(w0^2) ; ...
                      -2*W*s/(w0^2), (s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)]};
end



% They are compared in Figure [[fig:plant_compare_rotating_speed]].


freqs = logspace(-2, 1, 1000);

figure;
ax1 = subplot(2, 2, 1);
hold on;
for W_i = 1:length(Ws)
    plot(freqs, abs(squeeze(freqresp(Gs{W_i}(1,1), freqs))), ...
         'DisplayName', sprintf('$\\Omega = %.1f \\omega_0 $', Ws(W_i)/w0))
end
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [m/N]');
legend('location', 'southwest');
title('$d_u/F_u$, $d_v/F_v$');

ax3 = subplot(2, 2, 3);
hold on;
for W_i = 1:length(Ws)
    plot(freqs, 180/pi*angle(squeeze(freqresp(Gs{W_i}(1,1), freqs))))
end
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;

ax2 = subplot(2, 2, 2);
hold on;
for W_i = 1:length(Ws)
    plot(freqs, abs(squeeze(freqresp(Gs{W_i}(2,1), freqs))))
end
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [m/N]');
title('$d_u/F_v$, $d_v/F_u$');

ax4 = subplot(2, 2, 4);
hold on;
for W_i = 1:length(Ws)
    plot(freqs, 180/pi*angle(squeeze(freqresp(Gs{W_i}(2,1), freqs))))
end
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;

linkaxes([ax1,ax2,ax3,ax4],'x');
xlim([freqs(1), freqs(end)]);
linkaxes([ax1,ax2],'y');
