function [Ts, As] = chainDH(dh_table, q, verbose)
%CHAINDH Chain multiple DH links (standard DH) into transforms.
%   dh_table is N×4: [a alpha d theta_offset]
%   q is 1×N or N×1 of joint angles.
%   Ts is (N+1)×1 cell: Ts{1}=I, Ts{i+1}=T_0^i.
%   As is N×1 cell: As{i}=A_{i-1}^i.

if nargin < 3
    verbose = false;
end

n = size(dh_table, 1);
q = q(:).';
if numel(q) ~= n
    error('chainDH:InvalidQSize', 'Expected q to have %d elements, got %d.', n, numel(q));
end

Ts = cell(n+1, 1);
As = cell(n, 1);
Ts{1} = eye(4);

for i = 1:n
    a = dh_table(i, 1);
    alpha = dh_table(i, 2);
    d = dh_table(i, 3);
    theta_offset = dh_table(i, 4);
    theta = q(i) + theta_offset;

    As{i} = dh(a, alpha, d, theta, verbose);
    Ts{i+1} = Ts{i} * As{i};

    if verbose
        tprint(Ts{i+1}, sprintf('T_%d', i));
    end
end
end
