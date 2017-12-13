function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by value iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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
    
% --------------------------------- EMIEL'S CODE ---------------------------------

    numberOfCells = size(P, 1);
    numberOfInputs = size(P, 3);
    
    tres = 1e-50; % is this specified?

    %% INITIALIZE MATRICES
    J = zeros(numberOfCells, 1); % 1 x MN for the optimal cost-to-go, final cost 0??
    costToGo = zeros(numberOfCells, 1);
    uOpt = zeros(numberOfCells, 1); % 1 x MN for the indices of the optimal 
                                    % control inputs, note that the indices
                                    % can never be zero, therefore this 
                                    % initializations allows for checking.

    %% SOME VARIABLES
    numOfIt = 0;
    numOfItMax = 1000;
    converged = 0;

    %% VALUE ITERATION
    while ~converged

        numOfIt = numOfIt + 1; % increment iterations
        parSum = zeros(numberOfCells, numberOfInputs);

        for destCell = 1:numberOfCells
            parSum = parSum + squeeze(P(:,destCell,:)).*costToGo(destCell);
        end

        [costToGo, uOpt] = min(G + parSum, [], 2);
        
        if max(max(abs(J-costToGo))) < tres || numOfIt >= numOfItMax 
            converged = 1;
            J = costToGo;
        else
            J = costToGo;
        end
    end

    fprintf('Algorithm converged after %d iterations!\n', numOfIt);

    J_opt = J';
    u_opt_ind = uOpt';

% ------------------------------- EO EMIEL'S CODE --------------------------------
end

