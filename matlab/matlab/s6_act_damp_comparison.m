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



% DVF

Gdvf = (s/k)/(((s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2))^2 + (2*W*s/(w0^2))^2) * ...
       [(s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2), 2*W*s/(w0^2) ; ...
        -2*W*s/(w0^2), (s^2)/(w0^2) + 2*xi*s/w0 + 1 - (W^2)/(w0^2)];

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

set(gca,'ColorOrderIndex',3);
plot(real(pole(Gdvf)),  imag(pole(Gdvf)), 'x', ...
     'DisplayName', 'DVF');
set(gca,'ColorOrderIndex',3);
plot(real(tzero(Gdvf)),  imag(tzero(Gdvf)), 'o', ...
     'HandleVisibility', 'off');
for g = gains
    Kdvf = g*eye(2);
    cl_poles = pole(feedback(Gdvf, Kdvf));
    set(gca,'ColorOrderIndex',3);
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


%% IFF with pseudo integrators
gains = linspace(0, (w0^2/W^2 - 1)*wi, 100);
opt_zeta_iff = 0;
opt_gain_iff = 0;

for g = gains
    Kiff = (g/(wi+s))*eye(2);

    [w, zeta] = damp(minreal(feedback(Giff, Kiff)));

    if min(zeta) > opt_zeta_iff && all(zeta > 0)
      opt_zeta_iff = min(zeta);
      opt_gain_iff = g;
    end
end

%% IFF with Parallel Stiffness
gains = logspace(-2, 4, 100);
opt_zeta_kp = 0;
opt_gain_kp = 0;

for g = gains
    Kiff = g/s*eye(2);

    [w, zeta] = damp(minreal(feedback(Giff_kp, Kiff)));

    if min(zeta) > opt_zeta_kp && all(zeta > 0)
      opt_zeta_kp = min(zeta);
      opt_gain_kp = g;
    end
end

%% Direct Velocity Feedback
gains = logspace(0, 2, 100);
opt_zeta_dvf = 0;
opt_gain_dvf = 0;

for g = gains
    Kdvf = g*eye(2);

    [w, zeta] = damp(minreal(feedback(Gdvf, Kdvf)));

    if min(zeta) > opt_zeta_dvf && all(zeta > 0) && min(zeta) < 0.85
      opt_zeta_dvf = min(zeta);
      opt_gain_dvf = g;
    end
end

% Transmissibility
% <<sec:comp_transmissibilty>>


open('rotating_frame.slx');

% Open Loop                                                       :ignore:

Kdvf = tf(zeros(2));
Kiff = tf(zeros(2));

kp = 0;
cp = 0;

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/dw'], 1, 'input');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/Meas'], 1, 'output');  io_i = io_i + 1;

Tol = linearize(mdl, io, 0);

%% Input/Output definition
Tol.InputName  = {'Dwx', 'Dwy'};
Tol.OutputName = {'Dx', 'Dy'};

% Pseudo Integrator IFF                                           :ignore:

kp = 0;
cp = 0;

Kdvf = tf(zeros(2));

Kiff = opt_gain_iff/(wi + s)*tf(eye(2));

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/dw'], 1, 'input');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/Meas'], 1, 'output');  io_i = io_i + 1;

Tiff = linearize(mdl, io, 0);

%% Input/Output definition
Tiff.InputName  = {'Dwx', 'Dwy'};
Tiff.OutputName = {'Dx', 'Dy'};

% IFF With parallel Stiffness                                     :ignore:

kp = 5*m*W^2;
cp = 0.01;

Kiff = opt_gain_kp/s*tf(eye(2));

Kdvf = tf(zeros(2));

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/dw'], 1, 'input');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/Meas'], 1, 'output');  io_i = io_i + 1;

Tiff_kp = linearize(mdl, io, 0);

%% Input/Output definition
Tiff_kp.InputName  = {'Dwx', 'Dwy'};
Tiff_kp.OutputName = {'Dx', 'Dy'};

% DVF                                                             :ignore:

kp = 0;
cp = 0;

Kiff = tf(zeros(2));

Kdvf = opt_gain_kp*tf(eye(2));

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/dw'], 1, 'input');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/Meas'], 1, 'output');  io_i = io_i + 1;

Tdvf = linearize(mdl, io, 0);

%% Input/Output definition
Tdvf.InputName  = {'Dwx', 'Dwy'};
Tdvf.OutputName = {'Dx', 'Dy'};

% Transmissibility                                                :ignore:

freqs = logspace(-2, 1, 1000);

figure;
hold on;
plot(freqs, abs(squeeze(freqresp(Tiff(1,1), freqs))), ...
     'DisplayName', 'IFF + HPF')
plot(freqs, abs(squeeze(freqresp(Tiff_kp(1,1), freqs))), ...
     'DisplayName', 'IFF + $k_p$')
plot(freqs, abs(squeeze(freqresp(Tdvf(1,1), freqs))), ...
     'DisplayName', 'DVF')
plot(freqs, abs(squeeze(freqresp(Tol(1,1), freqs))), 'k-', ...
     'DisplayName', 'Open-Loop')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
xlabel('Frequency [rad/s]'); ylabel('Transmissibility [m/m]');
legend('location', 'southwest');

% Open Loop                                                       :ignore:

Kdvf = tf(zeros(2));
Kiff = tf(zeros(2));

kp = 0;
cp = 0;

%% Name of the Simulink File
mdl = 'rotating_frame';

%% Input/Output definition
clear io; io_i = 1;
io(io_i) = linio([mdl, '/fd'], 1, 'input');  io_i = io_i + 1;
io(io_i) = linio([mdl, '/Meas'], 1, 'output');  io_i = io_i + 1;

Col = linearize(mdl, io, 0);

%% Input/Output definition
Col.InputName  = {'Fdx', 'Fdy'};
Col.OutputName = {'Dx', 'Dy'};

% Pseudo Integrator IFF                                           :ignore:

kp = 0;
cp = 0;

Kdvf = tf(zeros(2));

Kiff = opt_gain_iff/(wi + s)*tf(eye(2));

Ciff = linearize(mdl, io, 0);

%% Input/Output definition
Ciff.InputName  = {'Fdx', 'Fdy'};
Ciff.OutputName = {'Dx', 'Dy'};

% IFF With parallel Stiffness                                     :ignore:

kp = 5*m*W^2;
cp = 0.01;

Kiff = opt_gain_kp/s*tf(eye(2));

Kdvf = tf(zeros(2));

Ciff_kp = linearize(mdl, io, 0);

%% Input/Output definition
Ciff_kp.InputName  = {'Fdx', 'Fdy'};
Ciff_kp.OutputName = {'Dx', 'Dy'};

% DVF                                                             :ignore:

kp = 0;
cp = 0;

Kiff = tf(zeros(2));

Kdvf = opt_gain_kp*tf(eye(2));

Cdvf = linearize(mdl, io, 0);

%% Input/Output definition
Cdvf.InputName  = {'Fdx', 'Fdy'};
Cdvf.OutputName = {'Dx', 'Dy'};

% Compliance                                                      :ignore:

freqs = logspace(-2, 1, 1000);

figure;
hold on;
plot(freqs, abs(squeeze(freqresp(Ciff(1,1), freqs))), ...
     'DisplayName', 'IFF + HPF')
plot(freqs, abs(squeeze(freqresp(Ciff_kp(1,1), freqs))), ...
     'DisplayName', 'IFF + $k_p$')
plot(freqs, abs(squeeze(freqresp(Cdvf(1,1), freqs))), ...
     'DisplayName', 'DVF')
plot(freqs, abs(squeeze(freqresp(Col(1,1), freqs))), 'k-', ...
     'DisplayName', 'Open-Loop')
hold off;
set(gca, 'XScale', 'log'); set(gca, 'YScale', 'log');
xlabel('Frequency [rad/s]'); ylabel('Compliance [m/N]');
legend('location', 'southwest');
