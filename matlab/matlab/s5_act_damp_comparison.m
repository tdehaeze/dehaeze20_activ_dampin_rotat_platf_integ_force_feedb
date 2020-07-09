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



% The rotating speed is set to $\Omega = 0.1 \omega_0$.

W = 0.1*w0;

% Root Locus
% IFF with High Pass Filter

wi = 0.1*w0; % [rad/s]

Giff = 1/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
        [(s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)) + (2*W*s/(w0^2))^2, - (2*xi*s/w0 + 1)*2*W*s/(w0^2) ; ...
         (2*xi*s/w0 + 1)*2*W*s/(w0^2), (s^2/w0^2 - W^2/w0^2)*((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))+ (2*W*s/(w0^2))^2];



% IFF With parallel Stiffness

kp = 5*m*W^2;
k = k - kp;

w0p = sqrt((k + kp)/m);
xip = c/(2*sqrt((k+kp)*m));

Giff_kp = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
                   (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
                   (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2 ];

k = k + kp;

figure;

gains = logspace(-2, 2, 100);

hold on;
set(gca,'ColorOrderIndex',1);
plot(real(pole(Giff)),  imag(pole(Giff)), 'x', ...
     'DisplayName', 'IFF + LFP');
set(gca,'ColorOrderIndex',1);
plot(real(tzero(Giff)),  imag(tzero(Giff)), 'o', ...
     'HandleVisibility', 'off');
for g = gains
    Kiff = (g/(wi + s))*eye(2);
    cl_poles = pole(feedback(Giff, Kiff));
    set(gca,'ColorOrderIndex',1);
    plot(real(cl_poles), imag(cl_poles), '.', ...
         'HandleVisibility', 'off');
end

set(gca,'ColorOrderIndex',2);
plot(real(pole(Giff_kp)),  imag(pole(Giff_kp)), 'x', ...
     'DisplayName', 'IFF + $k_p$');
set(gca,'ColorOrderIndex',2);
plot(real(tzero(Giff_kp)),  imag(tzero(Giff_kp)), 'o', ...
     'HandleVisibility', 'off');
for g = gains
    Kiffa = (g/s)*eye(2);
    cl_poles = pole(feedback(Giff_kp, Kiffa));
    set(gca,'ColorOrderIndex',2);
    plot(real(cl_poles), imag(cl_poles), '.', ...
         'HandleVisibility', 'off');
end
hold off;
axis square;
xlim([-1.2, 0.05]); ylim([0, 1.25]);

xlabel('Real Part'); ylabel('Imaginary Part');
legend('location', 'northwest');

% Controllers - Optimal Gains
% In order to compare to three considered Active Damping techniques, gains that yield maximum damping of all the modes are computed for each case.


fun = @(g)computeSimultaneousDamping(g, Giff, (1/(wi+s))*eye(2));

[opt_gain_iff, opt_xi_iff] = fminsearch(fun, 0.5*(w0^2/W^2 - 1)*wi);
opt_xi_iff = 1/opt_xi_iff;

fun = @(g)computeSimultaneousDamping(g, Giff_kp, 1/s*eye(2));

[opt_gain_kp, opt_xi_kp] = fminsearch(fun, 2);
opt_xi_kp = 1/opt_xi_kp;

% Passive Damping - Critical Damping
% \begin{equation}
%   \xi = \frac{c}{2 \sqrt{km}}
% \end{equation}

% Critical Damping corresponds to to $\xi = 1$, and thus:
% \begin{equation}
%   c_{\text{crit}} = 2 \sqrt{km}
% \end{equation}


c_opt = 2*sqrt(k*m);

% Transmissibility And Compliance
% <<sec:comp_transmissibilty>>


open('rotating_frame.slx');

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/dw'], 1, 'input');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/fd'], 1, 'input');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/Meas'], 1, 'output');  io_i = io_i + 1;

% Open Loop                                                       :ignore:

Kiff = tf(zeros(2));

kp = 0;
cp = 0;

G_ol = linearize(mdl, io, 0);

%% Input/Output definition
G_ol.InputName  = {'Dwx', 'Dwy', 'Fdx', 'Fdy'};
G_ol.OutputName = {'Dx', 'Dy'};

% Passive Damping

kp = 0;
cp = 0;

c_old = c;
c = c_opt;

Kiff = tf(zeros(2));

G_pas = linearize(mdl, io, 0);

%% Input/Output definition
G_pas.InputName  = {'Dwx', 'Dwy', 'Fdx', 'Fdy'};
G_pas.OutputName = {'Dx', 'Dy'};

c = c_old;

% Pseudo Integrator IFF                                           :ignore:

kp = 0;
cp = 0;

Kiff = opt_gain_iff/(wi + s)*tf(eye(2));

G_iff = linearize(mdl, io, 0);

%% Input/Output definition
G_iff.InputName  = {'Dwx', 'Dwy', 'Fdx', 'Fdy'};
G_iff.OutputName = {'Dx', 'Dy'};

% IFF With parallel Stiffness                                     :ignore:

kp = 5*m*W^2;
cp = 0.01;

Kiff = opt_gain_kp/s*tf(eye(2));

G_kp = linearize(mdl, io, 0);

%% Input/Output definition
G_kp.InputName  = {'Dwx', 'Dwy', 'Fdx', 'Fdy'};
G_kp.OutputName = {'Dx', 'Dy'};

% Transmissibility                                                :ignore:

freqs = logspace(-2, 1, 1000);

figure;
hold on;
plot(freqs, abs(squeeze(freqresp(G_iff({'Dx'}, {'Dwx'}), freqs))), ...
     'DisplayName', 'IFF + HPF')
plot(freqs, abs(squeeze(freqresp(G_kp( {'Dx'}, {'Dwx'}), freqs))), ...
     'DisplayName', 'IFF + $k_p$')
plot(freqs, abs(squeeze(freqresp(G_pas({'Dx'}, {'Dwx'}), freqs))), ...
     'DisplayName', 'Passive')
plot(freqs, abs(squeeze(freqresp(G_ol( {'Dx'}, {'Dwx'}), freqs))), 'k-', ...
     'DisplayName', 'Open-Loop')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
ylim([1e-2, 3e1]);
xlabel('Frequency [rad/s]'); ylabel('Transmissibility [m/m]');
legend('location', 'southwest');

% Compliance                                                      :ignore:

freqs = logspace(-2, 1, 1000);

figure;
hold on;
plot(freqs, abs(squeeze(freqresp(G_iff({'Dx'}, {'Fdx'}), freqs))), ...
     'DisplayName', 'IFF + HPF')
plot(freqs, abs(squeeze(freqresp(G_kp( {'Dx'}, {'Fdx'}), freqs))), ...
     'DisplayName', 'IFF + $k_p$')
plot(freqs, abs(squeeze(freqresp(G_pas({'Dx'}, {'Fdx'}), freqs))), ...
     'DisplayName', 'Passive')
plot(freqs, abs(squeeze(freqresp(G_ol( {'Dx'}, {'Fdx'}), freqs))), 'k-', ...
     'DisplayName', 'Open-Loop')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
ylim([1e-2, 3e1]);
xlabel('Frequency [rad/s]'); ylabel('Compliance [m/N]');
legend('location', 'southwest');
