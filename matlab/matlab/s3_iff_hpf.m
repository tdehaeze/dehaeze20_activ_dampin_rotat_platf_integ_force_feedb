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

% Modified Integral Force Feedback Controller
% Let's modify the initial Integral Force Feedback Controller ; instead of using pure integrators, pseudo integrators (i.e. low pass filters) are used:
% \begin{equation}
%   K_{\text{IFF}}(s) = g\frac{1}{\omega_i + s} \begin{bmatrix}
%   1 & 0 \\
%   0 & 1
% \end{bmatrix}
% \end{equation}
% where $\omega_i$ characterize down to which frequency the signal is integrated.

% Let's arbitrary choose the following control parameters:

g = 2;
wi = 0.1*w0;

Kiff = (g/(wi+s))*eye(2);



% And the following rotating speed.

W = 0.1*w0;

Giff = 1/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
        [(s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)) + (2*W*s/(w0^2))^2, - (2*xi*s/w0 + 1)*2*W*s/(w0^2) ; ...
         (2*xi*s/w0 + 1)*2*W*s/(w0^2), (s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))+ (2*W*s/(w0^2))^2];



% The obtained Loop Gain is shown in Figure [[fig:loop_gain_modified_iff]].

freqs = logspace(-2, 1, 1000);

figure;

ax1 = subplot(2, 1, 1);
hold on;
plot(freqs, abs(squeeze(freqresp(Giff(1,1)*(g/s), freqs))))
plot(freqs, abs(squeeze(freqresp(Giff(1,1)*Kiff(1,1), freqs))))
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
set(gca, 'XTickLabel',[]); ylabel('Loop Gain');

ax2 = subplot(2, 1, 2);
hold on;
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff(1,1)*(g/s), freqs))), ...
     'DisplayName', 'IFF')
plot(freqs, 180/pi*angle(squeeze(freqresp(Giff(1,1)*Kiff(1,1), freqs))), ...
     'DisplayName', 'IFF + HPF')
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'lin');
xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');
yticks(-180:90:180);
ylim([-180 180]);
legend('location', 'southwest');
hold off;

linkaxes([ax1,ax2],'x');
xlim([freqs(1), freqs(end)]);

% Root Locus
% As shown in the Root Locus plot (Figure [[fig:root_locus_modified_iff]]), for some value of the gain, the system remains stable.


figure;

gains = logspace(-2, 4, 100);

ax1 = subplot(1, 2, 1);
hold on;
% Pure Integrator
set(gca,'ColorOrderIndex',1);
plot(real(pole(Giff)),  imag(pole(Giff)), 'x', 'DisplayName', 'IFF');
set(gca,'ColorOrderIndex',1);
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o', 'HandleVisibility', 'off');
for g = gains
    clpoles = pole(feedback(Giff, (g/s)*eye(2)));
    set(gca,'ColorOrderIndex',1);
    plot(real(clpoles), imag(clpoles), '.', 'HandleVisibility', 'off');
end
% Modified IFF
set(gca,'ColorOrderIndex',2);
plot(real(pole(Giff)),  imag(pole(Giff)), 'x', 'DisplayName', 'IFF + HPF');
set(gca,'ColorOrderIndex',2);
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o', 'HandleVisibility', 'off');
for g = gains
    clpoles = pole(feedback(Giff, (g/(wi+s))*eye(2)));
    set(gca,'ColorOrderIndex',2);
    plot(real(clpoles), imag(clpoles), '.', 'HandleVisibility', 'off');
end
hold off;
axis square;
xlim([-2, 0.5]); ylim([-1.25, 1.25]);
legend('location', 'northwest');
xlabel('Real Part'); ylabel('Imaginary Part');

ax2 = subplot(1, 2, 2);
hold on;
% Pure Integrator
set(gca,'ColorOrderIndex',1);
plot(real(pole(Giff)),  imag(pole(Giff)), 'x');
set(gca,'ColorOrderIndex',1);
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o');
for g = gains
    clpoles = pole(feedback(Giff, (g/s)*eye(2)));
    set(gca,'ColorOrderIndex',1);
    plot(real(clpoles), imag(clpoles), '.');
end
% Modified IFF
set(gca,'ColorOrderIndex',2);
plot(real(pole(Giff)),  imag(pole(Giff)), 'x');
set(gca,'ColorOrderIndex',2);
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o');
for g = gains
    clpoles = pole(feedback(Giff, (g/(wi+s))*eye(2)));
    set(gca,'ColorOrderIndex',2);
    plot(real(clpoles), imag(clpoles), '.');
end
hold off;
axis square;
xlim([-0.2, 0.1]); ylim([-0.15, 0.15]);
xlabel('Real Part'); ylabel('Imaginary Part');

% What is the optimal $\omega_i$ and $g$?
% In order to visualize the effect of $\omega_i$ on the attainable damping, the Root Locus is displayed in Figure [[fig:root_locus_wi_modified_iff]] for the following $\omega_i$:

wis = [0.01, 0.1, 0.5, 1]*w0; % [rad/s]

figure;

gains = logspace(-2, 4, 100);

ax1 = subplot(1, 2, 1);
hold on;
for wi_i = 1:length(wis)
    set(gca,'ColorOrderIndex',wi_i);
    wi = wis(wi_i);
    L(wi_i) = plot(nan, nan, '.', 'DisplayName', sprintf('$\\omega_i = %.2f \\omega_0$', wi./w0));
    for g = gains
        clpoles = pole(feedback(Giff, (g/(wi+s))*eye(2)));
        set(gca,'ColorOrderIndex',wi_i);
        plot(real(clpoles), imag(clpoles), '.');
    end
end
plot(real(pole(Giff)),  imag(pole(Giff)), 'kx');
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'ko');
hold off;
axis square;
xlim([-2.3, 0.1]); ylim([-1.2, 1.2]);
xticks([-2:1:2]); yticks([-2:1:2]);
legend(L, 'location', 'northwest');
xlabel('Real Part'); ylabel('Imaginary Part');

clear L

ax2 = subplot(1, 2, 2);
hold on;
for wi_i = 1:length(wis)
    set(gca,'ColorOrderIndex', wi_i);
    wi = wis(wi_i);
    for g = gains
        clpoles = pole(feedback(Giff, (g/(wi+s))*eye(2)));
        set(gca,'ColorOrderIndex', wi_i);
        plot(real(clpoles), imag(clpoles), '.');
    end
end
plot(real(pole(Giff)),  imag(pole(Giff)), 'kx');
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'ko');
hold off;
axis square;
xlim([-0.2, 0.1]); ylim([-0.15, 0.15]);
xticks([-0.2:0.1:0.1]); yticks([-0.2:0.1:0.2]);
xlabel('Real Part'); ylabel('Imaginary Part');



% For the controller
% \begin{equation}
%   K_{\text{IFF}}(s) = g\frac{1}{\omega_i + s} \begin{bmatrix}
%   1 & 0 \\
%   0 & 1
% \end{bmatrix}
% \end{equation}
% The gain at which the system becomes unstable is
% \begin{equation}
%   g_\text{max} = \omega_i \left( \frac{{\omega_0}^2}{\Omega^2} - 1 \right) \label{eq:iff_gmax}
% \end{equation}

% While it seems that small $\omega_i$ do allow more damping to be added to the system (Figure [[fig:root_locus_wi_modified_iff]]), the control gains may be limited to small values due to eqref:eq:iff_gmax thus reducing the attainable damping.


% There must be an optimum for $\omega_i$.
% To find the optimum, the gain that maximize the simultaneous damping of the mode is identified for a wide range of $\omega_i$ (Figure [[fig:mod_iff_damping_wi]]).

wis = logspace(-2, 1, 31)*w0; % [rad/s]

opt_zeta = zeros(1, length(wis)); % Optimal simultaneous damping
opt_gain = zeros(1, length(wis)); % Corresponding optimal gain

for wi_i = 1:length(wis)
    wi = wis(wi_i);
    gains = linspace(0, (w0^2/W^2 - 1)*wi, 100);

    for g = gains
        Kiff = (g/(wi+s))*eye(2);

        [w, zeta] = damp(minreal(feedback(Giff, Kiff)));

        if min(zeta) > opt_zeta(wi_i) && all(zeta > 0)
            opt_zeta(wi_i) = min(zeta);
            opt_gain(wi_i) = g;
        end
    end
end

figure;
yyaxis left
plot(wis, opt_zeta, '-o', 'DisplayName', '$\xi_{cl}$');
set(gca, 'YScale', 'lin');
ylim([0,1]);
ylabel('Attainable Damping Ratio $\xi$');

yyaxis right
hold on;
plot(wis, opt_gain, '-x', 'DisplayName', '$g_{opt}$');
plot(wis, wis*((w0/W)^2 - 1), '--', 'DisplayName', '$g_{max}$');
set(gca, 'YScale', 'lin');
ylim([0,10]);
ylabel('Controller gain $g$');

xlabel('$\omega_i/\omega_0$');
set(gca, 'XScale', 'log');
legend('location', 'northeast');
