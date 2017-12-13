function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by linear programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (1 x MN) matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (1 x MN) matrix containing the indices of the optimal control
%       	inputs for each element of the state space.

% put your code here
    
    % Some useful variables
    numberOfInputs = size(G, 2);   
    numberOfCells = size(G, 1);
    
    % Initialize f (- for minimize)
    f = -ones(numberOfCells-1, 1);
    
    % Identify terminal state
    for i = 1:numberOfCells
        if all(G(i,:) < 10e-2)
            terminalStateID = i;
        end
    end
    
    % Set up b
    b = G;
    b(terminalStateID,:) = [];
    b = reshape(b, numberOfInputs*(numberOfCells-1), 1);
    b(b==Inf) = 1e10;
    
    % Set up b
    A = zeros((numberOfCells-1)*numberOfInputs, numberOfCells-1);
    
    cellRange = 1:numberOfCells;
    cellRange(terminalStateID) = [];

    % Combine right elements of P
    for i = cellRange
        for u = 1:numberOfInputs
            for j = cellRange
                Ai = find(cellRange==i);
                Aj = find(cellRange==j);
                A((numberOfCells-1)*(u-1) + Ai, Aj) = -P(i, j, u);
            end
        end
    end

    % Take I into account
    for i = 1:length(cellRange)
        for u = 1:numberOfInputs
            A((numberOfCells-1)*(u-1) + i, i) = 1 + A((numberOfCells-1)*(u-1) + i, i); 
        end
    end

    J = linprog(f, A, b);

    % Insert terminal state in J_opt
    J = [J(1:(terminalStateID-1)); 0; J(terminalStateID:end)];
    
    % Find optimal control policy 
    u_opt = 5*ones(numberOfInputs, 1);

    tres = 1e-5;

    % Check for constraints
    for i = 1:numberOfCells
        for u = 1:numberOfInputs
            constraint = G(i, u) + squeeze(P(i, :, u))*J;
            if constraint - J(i) < tres
                u_opt(i) = u;
            end
        end
    end
 
    u_opt(terminalStateID) = 1;
    J_opt = J;
    u_opt_ind = u_opt;
end

