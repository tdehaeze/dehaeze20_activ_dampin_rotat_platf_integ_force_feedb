% Attainable Damping as a function of $k_p$

tic;
alphas = logspace(-2, 0, 10);
gains = linspace(0.5, 2.5, 100);

opt_zeta = zeros(1, length(alphas)); % Optimal simultaneous damping
opt_gain = zeros(1, length(alphas)); % Corresponding optimal gain

for alpha_i = 1:length(alphas)
    kp = alphas(alpha_i);
    k = 1 - alphas(alpha_i);

    w0p = sqrt((k + kp)/m);
    xip = c/(2*sqrt((k+kp)*m));

    Giff = 1/( (s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2)^2 + (2*(s/w0p)*(W/w0p))^2 ) * [ ...
        (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2, -(2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p));
        (2*xip*s/w0p + k/(k + kp))*(2*(s/w0p)*(W/w0p)), (s^2/w0p^2 + kp/(k + kp) - W^2/w0p^2)*(s^2/w0p^2 + 2*xip*s/w0p + 1 - W^2/w0p^2) + (2*(s/w0p)*(W/w0p))^2];

    for g = gains
        Kiff = g/s*eye(2);

        [w, zeta] = damp(minreal(feedback(Giff, Kiff)));

        if min(zeta) > opt_zeta(alpha_i) && all(zeta > 0)
            opt_zeta(alpha_i) = min(zeta);
            opt_gain(alpha_i) = g;
        end
    end
end
toc

figure;
yyaxis left
plot(alphas, opt_zeta, '-o', 'DisplayName', '$\xi_{cl}$');
set(gca, 'YScale', 'lin');
ylim([0,1]);
ylabel('Attainable Damping Ratio $\xi$');

yyaxis right
hold on;
plot(alphas, opt_gain, '-x', 'DisplayName', '$g_{opt}$');
set(gca, 'YScale', 'lin');
ylim([0,3]);
ylabel('Controller gain $g$');

xlabel('$\alpha$');
set(gca, 'XScale', 'log');
legend('location', 'northeast');



% Alternative using fminserach

alphas = logspace(-2, 0, 100);

opt_zeta = zeros(1, length(alphas)); % Optimal simultaneous damping
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
    opt_zeta(alpha_i) = 1/xi_opt;
    opt_gain(alpha_i) = g_opt;
end
