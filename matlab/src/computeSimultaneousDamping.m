% Compute Damping

function [xi_min] = computeSimultaneousDamping(g, G, K)
    [w, xi] = damp(minreal(feedback(G, g*K)));
    xi_min = 1/min(xi);

    if xi_min < 0
      xi_min = 1e8;
    end
end
