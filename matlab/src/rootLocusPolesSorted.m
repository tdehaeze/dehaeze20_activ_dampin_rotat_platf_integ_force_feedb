function [poles] = rootLocusPolesSorted(G, K, gains, args)
% rootLocusPolesSorted -
%
% Syntax: [poles] = rootLocusPolesSorted(G, K, gains, args)
%
% Inputs:
%    - G, K, gains, args -
%
% Outputs:
%    - poles -

arguments
    G
    K
    gains
    args.minreal double {mustBeNumericOrLogical} = false
    args.p_half  double {mustBeNumericOrLogical} = false
    args.d_max   double {mustBeNumeric} = -1
end

if args.minreal
    p1 = pole(minreal(feedback(G, gains(1)*K)));
    [~, i_uniq] = uniquetol([real(p1), imag(p1)], 1e-10, 'ByRows', true);
    p1 = p1(i_uniq);

    poles = zeros(length(p1), length(gains));
    poles(:, 1) = p1;
else
    p1 = pole(feedback(G, gains(1)*K));
    [~, i_uniq] = uniquetol([real(p1), imag(p1)], 1e-10, 'ByRows', true);
    p1 = p1(i_uniq);

    poles = zeros(length(p1), length(gains));
    poles(:, 1) = p1;
end

if args.minreal
    p2 = pole(minreal(feedback(G, gains(2)*K)));
    [~, i_uniq] = uniquetol([real(p2), imag(p2)], 1e-10, 'ByRows', true);
    p2 = p2(i_uniq);
    poles(:, 2) = p2;
else
    p2 = pole(feedback(G, gains(2)*K));
    [~, i_uniq] = uniquetol([real(p2), imag(p2)], 1e-10, 'ByRows', true);
    p2 = p2(i_uniq);
    poles(:, 2) = p2;
end

for g_i = 3:length(gains)
    % Estimated value of the poles
    poles_est = poles(:, g_i-1) + (poles(:, g_i-1) - poles(:, g_i-2))*(gains(g_i) - gains(g_i-1))/(gains(g_i-1) - gains(g_i - 2));

    % New values for the poles
    poles_gi = pole(feedback(G, gains(g_i)*K));
    [~, i_uniq] = uniquetol([real(poles_gi), imag(poles_gi)], 1e-10, 'ByRows', true);
    poles_gi = poles_gi(i_uniq);

    % Array of distances between all the poles
    poles_dist = sqrt((poles_est-poles_gi.').*conj(poles_est-poles_gi.'));

    % Get indices corresponding to distances from lowest to highest
    [~, c] = sort(min(poles_dist));

    as = 1:length(poles_gi);

    % for each column of poles_dist corresponding to the i'th pole
    % with closest previous poles
    for p_i = c
        % Get the indice a_i of the previous pole that is the closest
        % to pole c(p_i)
        [~, a_i] = min(poles_dist(:, p_i));

        poles(as(a_i), g_i) = poles_gi(p_i);

        % Remove old poles that are already matched
        % poles_gi(as(a_i), :) = [];
        poles_dist(a_i, :) = [];
        as(a_i) = [];
    end
end

if args.d_max > 0
    poles = poles(max(abs(poles(:, 2:end) - poles(:, 1:end-1))') > args.d_max, :);
end

if args.p_half
    poles = poles(1:round(end/2), :);
end

[~, s_p] = sort(imag(poles(:,1)), 'descend');
poles = poles(s_p, :);

poles = poles.';
