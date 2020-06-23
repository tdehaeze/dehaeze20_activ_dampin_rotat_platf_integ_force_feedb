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

% Comparison of the Analytical Model and the Simscape Model
% The rotating speed is set to $\Omega = 0.1 \omega_0$.

W = 0.1*w0;

Kiff = tf(zeros(2));
Kdvf = tf(zeros(2));

open('rotating_frame.slx');



% And the transfer function from $[F_u, F_v]$ to $[v_u, v_v]$ is identified using the Simscape model.

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/K'], 1, 'openinput');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/G'], 1, 'openoutput'); io_i = io_i + 1;

Gdvf = linearize(mdl, io, 0);

%% Input/Output definition
Gdvf.InputName  = {'Fu', 'Fv'};
Gdvf.OutputName = {'Vu', 'Vv'};



% The same transfer function from $[F_u, F_v]$ to $[v_u, v_v]$ is written down from the analytical model.

Gdvf_th = (s/k)/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
          [(s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2), 2*W*s/(w0^2) ; ...
           -2*W*s/(w0^2), (s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)];

Gdvf_th.InputName  = {'Fu', 'Fv'};
Gdvf_th.OutputName = {'vu', 'vv'};



% The two are compared in Figure [[fig:plant_iff_comp_simscape_analytical]] and found to perfectly match.


freqs = logspace(-1, 1, 1000);

figure;
ax1 = subplot(2, 2, 1);
hold on;
plot(freqs, abs(squeeze(freqresp(Gdvf(1,1), freqs))), '-')
plot(freqs, abs(squeeze(freqresp(Gdvf_th(1,1), freqs))), '--')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [$\frac{m/s}{N}$]');
title('$v_u/F_u$, $v_v/F_v$');

ax3 = subplot(2, 2, 3);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(Gdvf(1,1), freqs))), '-')
plot(freqs, 180/pi*angle(squeeze(freqresp(Gdvf_th(1,1), freqs))), '--')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;

ax2 = subplot(2, 2, 2);
hold on;
plot(freqs, abs(squeeze(freqresp(Gdvf(1,2), freqs))), '-')
plot(freqs, abs(squeeze(freqresp(Gdvf_th(1,2), freqs))), '--')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [$\frac{m/s}{N}$]');
title('$v_u/F_v$, $v_v/F_u$');

ax4 = subplot(2, 2, 4);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(Gdvf(1,2), freqs))), '-')
plot(freqs, 180/pi*angle(squeeze(freqresp(Gdvf_th(1,2), freqs))), '--')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;

linkaxes([ax1,ax2,ax3,ax4],'x');
xlim([freqs(1), freqs(end)]);

linkaxes([ax1,ax2],'y');

% Root Locus
% The Decentralized Direct Velocity Feedback controller consist of a pure gain on the diagonal:
% \begin{equation}
%   K_{\text{DVF}}(s) = g \begin{bmatrix}
%   1 & 0 \\
%   0 & 1
% \end{bmatrix}
% \end{equation}

% The corresponding Root Locus plots for the following rotating speeds are shown in Figure [[fig:root_locus_dvf]].

Ws = [0, 0.2, 0.7, 1.1]*w0; % Rotating Speeds [rad/s]



% It is shown that for rotating speed $\Omega < \omega_0$, the closed loop system is unconditionally stable and arbitrary damping can be added to the poles.

gains = logspace(-2, 1, 100);

figure;
hold on;
for W_i = 1:length(Ws)
    W = Ws(W_i);

    Gdvf = (s/k)/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
           [(s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2), 2*W*s/(w0^2) ; ...
            -2*W*s/(w0^2), (s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)];

    set(gca,'ColorOrderIndex',W_i);
    plot(real(pole(Gdvf)),  imag(pole(Gdvf)), 'x', ...
         'DisplayName', sprintf('$\\Omega = %.2f \\omega_0 $', W/w0));

    set(gca,'ColorOrderIndex',W_i);
    plot(real(tzero(Gdvf)),  imag(tzero(Gdvf)), 'o', ...
         'HandleVisibility', 'off');

    for g = gains
        set(gca,'ColorOrderIndex',W_i);
        cl_poles = pole(feedback(Gdvf, g*eye(2)));

        plot(real(cl_poles), imag(cl_poles), '.', ...
             'HandleVisibility', 'off');
    end
end
hold off;
axis square;
xlim([-2, 0.5]); ylim([0, 2.5]);

xlabel('Real Part'); ylabel('Imaginary Part');
legend('location', 'northwest');
