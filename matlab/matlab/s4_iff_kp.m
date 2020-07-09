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
% The same transfer function from $[F_u, F_v]$ to $[f_u, f_v]$ is written down from the analytical model.

W = 0.1*w0; % [rad/s]

kp = 1.5*m*W^2;
cp = 0;

Kiff = tf(zeros(2));

open('rotating_frame.slx');

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

w0p = sqrt((k + kp)/m);
xip = c/(2*sqrt((k+kp)*m));

Giff_th = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
                   (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
                   (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2 ];
Giff_th.InputName  = {'Fu', 'Fv'};
Giff_th.OutputName = {'fu', 'fv'};

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

% Effect of the parallel stiffness on the IFF plant
% The rotation speed is set to $\Omega = 0.1 \omega_0$.

W = 0.1*w0; % [rad/s]



% And the IFF plant (transfer function from $[F_u, F_v]$ to $[f_u, f_v]$) is identified in three different cases:
% - without parallel stiffness
% - with a small parallel stiffness $k_p < m \Omega^2$
% - with a large parallel stiffness $k_p > m \Omega^2$

% The results are shown in Figure [[fig:plant_iff_kp]].

% One can see that for $k_p > m \Omega^2$, the systems shows alternating complex conjugate poles and zeros.


kp = 0;

w0p = sqrt((k + kp)/m);
xip = c/(2*sqrt((k+kp)*m));

Giff = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
    (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
    (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2];

kp = 0.5*m*W^2;
k = 1 - kp;

w0p = sqrt((k + kp)/m);
xip = c/(2*sqrt((k+kp)*m));

Giff_s = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
    (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
    (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2];

kp = 1.5*m*W^2;
k = 1 - kp;

w0p = sqrt((k + kp)/m);
xip = c/(2*sqrt((k+kp)*m));

Giff_l = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
    (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
    (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2];

freqs = logspace(-2, 1, 1000);

figure;

ax1 = subplot(2, 1, 1);
hold on;
plot(freqs, abs(squeeze(freqresp(Giff(1,1),   freqs))), 'k-')
plot(freqs, abs(squeeze(freqresp(Giff_s(1,1), freqs))), 'k--')
plot(freqs, abs(squeeze(freqresp(Giff_l(1,1), freqs))), 'k:')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Magnitude [N/N]');
ylim([1e-5, 2e1]);

ax2 = subplot(2, 1, 2);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff(1,1),   freqs))), 'k-', ...
     'DisplayName', '$k_p = 0$')
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff_s(1,1), freqs))), 'k--', ...
     'DisplayName', '$k_p < m\Omega^2$')
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff_l(1,1), freqs))), 'k:', ...
     'DisplayName', '$k_p > m\Omega^2$')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
hold off;
legend('location', 'southwest');

linkaxes([ax1,ax2],'x');
xlim([freqs(1), freqs(end)]);

% IFF when adding a spring in parallel
% In Figure [[fig:root_locus_iff_kp]] is displayed the Root Locus in the three considered cases with
% \begin{equation}
%   K_{\text{IFF}} = \frac{g}{s} \begin{bmatrix}
%   1 & 0 \\
%   0 & 1
% \end{bmatrix}
% \end{equation}

% One can see that for $k_p > m \Omega^2$, the root locus stays in the left half of the complex plane and thus the control system is unconditionally stable.

% Thus, decentralized IFF controller with pure integrators can be used if:
% \begin{equation}
%   k_{p} > m \Omega^2
% \end{equation}


figure;

gains = logspace(-2, 2, 100);

subplot(1,2,1);
hold on;
set(gca,'ColorOrderIndex',1);
plot(real(pole(Giff)),  imag(pole(Giff)), 'x', ...
     'DisplayName', '$k_p = 0$');
set(gca,'ColorOrderIndex',1);
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o', ...
     'HandleVisibility', 'off');
for g = gains
    cl_poles = pole(feedback(Giff, (g/s)*eye(2)));
    set(gca,'ColorOrderIndex',1);
    plot(real(cl_poles), imag(cl_poles), '.', ...
         'HandleVisibility', 'off');
end

set(gca,'ColorOrderIndex',2);
plot(real(pole(Giff_s)),  imag(pole(Giff_s)), 'x', ...
     'DisplayName', '$k_p < m\Omega^2$');
set(gca,'ColorOrderIndex',2);
plot(real(tzero(Giff_s)),  imag(tzero(Giff_s)), 'o', ...
     'HandleVisibility', 'off');
for g = gains
    cl_poles = pole(feedback(Giff_s, (g/s)*eye(2)));
    set(gca,'ColorOrderIndex',2);
    plot(real(cl_poles), imag(cl_poles), '.', ...
         'HandleVisibility', 'off');
end

set(gca,'ColorOrderIndex',3);
plot(real(pole(Giff_l)),  imag(pole(Giff_l)), 'x', ...
     'DisplayName', '$k_p > m\Omega^2$');
set(gca,'ColorOrderIndex',3);
plot(real(tzero(Giff_l)),  imag(tzero(Giff_l)), 'o', ...
     'HandleVisibility', 'off');
for g = gains
    set(gca,'ColorOrderIndex',3);
    cl_poles = pole(feedback(Giff_l, (g/s)*eye(2)));
    plot(real(cl_poles), imag(cl_poles), '.', ...
         'HandleVisibility', 'off');
end
hold off;
axis square;
xlim([-1, 0.2]); ylim([0, 1.2]);

xlabel('Real Part'); ylabel('Imaginary Part');
legend('location', 'northwest');

subplot(1,2,2);
hold on;
set(gca,'ColorOrderIndex',1);
plot(real(pole(Giff)),  imag(pole(Giff)), 'x');
set(gca,'ColorOrderIndex',1);
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o');
for g = gains
    cl_poles = pole(feedback(Giff, (g/s)*eye(2)));
    set(gca,'ColorOrderIndex',1);
    plot(real(cl_poles), imag(cl_poles), '.');
end

set(gca,'ColorOrderIndex',2);
plot(real(pole(Giff_s)),  imag(pole(Giff_s)), 'x');
set(gca,'ColorOrderIndex',2);
plot(real(tzero(Giff_s)),  imag(tzero(Giff_s)), 'o');
for g = gains
    cl_poles = pole(feedback(Giff_s, (g/s)*eye(2)));
    set(gca,'ColorOrderIndex',2);
    plot(real(cl_poles), imag(cl_poles), '.');
end

set(gca,'ColorOrderIndex',3);
plot(real(pole(Giff_l)),  imag(pole(Giff_l)), 'x');
set(gca,'ColorOrderIndex',3);
plot(real(tzero(Giff_l)),  imag(tzero(Giff_l)), 'o');
for g = gains
    set(gca,'ColorOrderIndex',3);
    cl_poles = pole(feedback(Giff_l, (g/s)*eye(2)));
    plot(real(cl_poles), imag(cl_poles), '.');
end
hold off;
axis square;
xlim([-0.04, 0.06]); ylim([0, 0.1]);

xlabel('Real Part'); ylabel('Imaginary Part');

% Effect of $k_p$ on the attainable damping
% However, having large values of $k_p$ may decrease the attainable damping.

% To study the second point, Root Locus plots for the following values of $k_p$ are shown in Figure [[fig:root_locus_iff_kps]].

kps = [2, 20, 40]*m*W^2;



% It is shown that large values of $k_p$ decreases the attainable damping.

figure;

gains = logspace(-2, 4, 500);

hold on;
for kp_i = 1:length(kps)
    kp = kps(kp_i);
    k = 1 - kp;

    w0p = sqrt((k + kp)/m);
    xip = c/(2*sqrt((k+kp)*m));

    Giff = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
        (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
        (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2 ];

    set(gca,'ColorOrderIndex',kp_i);
    plot(real(pole(Giff)),  imag(pole(Giff)), 'x', ...
         'DisplayName', sprintf('$k_p = %.1f m \\Omega^2$', kp/(m*W^2)));
    set(gca,'ColorOrderIndex',kp_i);
    plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o', ...
         'HandleVisibility', 'off');
    for g = gains
        Kiffa = (g/s)*eye(2);
        cl_poles = pole(feedback(Giff, Kiffa));
        set(gca,'ColorOrderIndex',kp_i);
        plot(real(cl_poles), imag(cl_poles), '.', ...
             'HandleVisibility', 'off');
    end
end
hold off;
axis square;
xlim([-1.2, 0.2]); ylim([0, 1.4]);

xlabel('Real Part'); ylabel('Imaginary Part');
legend('location', 'northwest');

alphas = logspace(-2, 0, 100);

opt_xi = zeros(1, length(alphas)); % Optimal simultaneous damping
opt_gain = zeros(1, length(alphas)); % Corresponding optimal gain

Kiff = 1/s*eye(2);

for alpha_i = 1:length(alphas)
    kp = alphas(alpha_i);
    k = 1 - alphas(alpha_i);

    w0p = sqrt((k + kp)/m);
    xip = c/(2*sqrt((k+kp)*m));

    Giff = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
        (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
        (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2];

    fun = @(g)computeSimultaneousDamping(g, Giff, Kiff);

    [g_opt, xi_opt] = fminsearch(fun, 2);
    opt_xi(alpha_i) = 1/xi_opt;
    opt_gain(alpha_i) = g_opt;
end

figure;
yyaxis left
plot(alphas, opt_xi, '-', 'DisplayName', '$\xi_{cl}$');
set(gca, 'YScale', 'lin');
ylim([0,1]);
ylabel('Attainable Damping Ratio $\xi$');

yyaxis right
hold on;
plot(alphas, opt_gain, '-', 'DisplayName', '$g_{opt}$');
set(gca, 'YScale', 'lin');
ylim([0,2.5]);
ylabel('Controller gain $g$');

xlabel('$\alpha$');
set(gca, 'XScale', 'log');
legend('location', 'northeast');
