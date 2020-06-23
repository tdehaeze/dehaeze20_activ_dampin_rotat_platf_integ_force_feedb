%% Clear Workspace and Close figures
clear; close all; clc;

%% Intialize Laplace variable
s = zpk('s');

% Plant Parameters
% Let's define initial values for the model.

k = 1;    % Actuator Stiffness [N/m]
c = 0.05; % Actuator Damping [N/(m/s)]
m = 1;    % Payload mass [kg]

xi = c/(2*sqrt(k*m));
w0 = sqrt(k/m); % [rad/s]

kp = 0; % [N/m]
cp = 0; % [N/(m/s)]

% Simscape Model
% The rotation speed is set to $\Omega = 0.1 \omega_0$.

W = 0.1*w0; % [rad/s]

Kiff = tf(zeros(2));
Kdvf = tf(zeros(2));

open('rotating_frame.slx');



% And the transfer function from $[F_u, F_v]$ to $[f_u, f_v]$ is identified using the Simscape model.

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/K'], 1, 'openinput');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/G'], 2, 'openoutput'); io_i = io_i + 1;

Giff = linearize(mdl, io, 0);

%% Input/Output definition
Giff.InputName  = {'Fu', 'Fv'};
Giff.OutputName = {'fu', 'fv'};

% Comparison of the Analytical Model and the Simscape Model
% The same transfer function from $[F_u, F_v]$ to $[f_u, f_v]$ is written down from the analytical model.

Giff_th = 1/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
          [(s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)) + (2*W*s/(w0^2))^2, - (2*xi*s/w0 + 1)*2*W*s/(w0^2) ; ...
           (2*xi*s/w0 + 1)*2*W*s/(w0^2), (s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))+ (2*W*s/(w0^2))^2];



% The two are compared in Figure [[fig:plant_iff_comp_simscape_analytical]] and found to perfectly match.


freqs = logspace(-1, 1, 1000);

figure;
ax1 = subplot(2, 2, 1);
hold on;
plot(freqs, abs(squeeze(freqresp(Giff(1,1), freqs))), '-')
plot(freqs, abs(squeeze(freqresp(Giff_th(1,1), freqs))), '--')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [N/N]');
title('$f_u/F_u$, $f_v/F_v$');

ax3 = subplot(2, 2, 3);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff(1,1), freqs))), '-')
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff_th(1,1), freqs))), '--')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;

ax2 = subplot(2, 2, 2);
hold on;
plot(freqs, abs(squeeze(freqresp(Giff(1,2), freqs))), '-')
plot(freqs, abs(squeeze(freqresp(Giff_th(1,2), freqs))), '--')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [N/N]');
title('$f_u/F_v$, $f_v/F_u$');

ax4 = subplot(2, 2, 4);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff(1,2), freqs))), '-', ...
     'DisplayName', 'Simscape')
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff_th(1,2), freqs))), '--', ...
     'DisplayName', 'Analytical')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;
legend('location', 'northeast');

linkaxes([ax1,ax2,ax3,ax4],'x');
xlim([freqs(1), freqs(end)]);
linkaxes([ax1,ax2],'y');

% Effect of the rotation speed
% The transfer functions from $[F_u, F_v]$ to $[f_u, f_v]$ are identified for the following rotating speeds.

Ws = [0, 0.2, 0.7, 1.1]*w0; % Rotating Speeds [rad/s]

Gsiff = {zeros(2, 2, length(Ws))};

for W_i = 1:length(Ws)
    W = Ws(W_i);

    Gsiff(:, :, W_i) = {1/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
                      [(s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)) + (2*W*s/(w0^2))^2, - (2*xi*s/w0 + 1)*2*W*s/(w0^2) ; ...
                       (2*xi*s/w0 + 1)*2*W*s/(w0^2), (s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))+ (2*W*s/(w0^2))^2]};
end



% The obtained transfer functions are shown in Figure [[fig:plant_iff_compare_rotating_speed]].

freqs = logspace(-2, 1, 1000);

figure;

ax1 = subplot(2, 1, 1);
hold on;
for W_i = 1:length(Ws)
    plot(freqs, abs(squeeze(freqresp(Gsiff{W_i}(1,1), freqs))), ...
         'DisplayName', sprintf('$\\Omega = %.1f \\omega_0 $', Ws(W_i)/w0))
end
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [N/N]');
legend('location', 'southeast');

ax2 = subplot(2, 1, 2);
hold on;
for W_i = 1:length(Ws)
    plot(freqs, 180/pi*angle(squeeze(freqresp(Gsiff{W_i}(1,1), freqs))))
end
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;

linkaxes([ax1,ax2],'x');
xlim([freqs(1), freqs(end)]);

% Decentralized Integral Force Feedback
% The decentralized IFF controller consists of pure integrators:
% \begin{equation}
%   \bm{K}_{\text{IFF}}(s) = \frac{g}{s} \begin{bmatrix}
%     1 & 0 \\
%     0 & 1
%   \end{bmatrix}
% \end{equation}

% The Root Locus (evolution of the poles of the closed loop system in the complex plane as a function of $g$) is shown in Figure [[fig:root_locus_pure_iff]].
% It is shown that for non-null rotating speed, one pole is bound to the right-half plane, and thus the closed loop system is unstable.


figure;

gains = logspace(-2, 4, 100);

hold on;
for W_i = 1:length(Ws)
    set(gca,'ColorOrderIndex',W_i);
    plot(real(pole(Gsiff{W_i})),  imag(pole(Gsiff{W_i})), 'x', ...
         'DisplayName', sprintf('$\\Omega = %.1f \\omega_0 $', Ws(W_i)/w0));
    set(gca,'ColorOrderIndex',W_i);
    plot(real(tzero(Gsiff{W_i})),  imag(tzero(Gsiff{W_i})), 'o', ...
         'HandleVisibility', 'off');
    for g = gains
        set(gca,'ColorOrderIndex',W_i);
        cl_poles = pole(feedback(Gsiff{W_i}, g/s*eye(2)));
        plot(real(cl_poles), imag(cl_poles), '.', ...
             'HandleVisibility', 'off');
    end
end
hold off;
axis square;
xlim([-2, 0.5]); ylim([0, 2.5]);

xlabel('Real Part'); ylabel('Imaginary Part');
legend('location', 'northwest');
