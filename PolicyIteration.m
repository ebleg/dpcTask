function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by policy iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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
%% INITIALIZE
   numberOfCells = size(P, 1);
   numberOfInputs = size(P, 3);
   controlSpaceID = 1:numberOfInputs;

   
   uOpt = ones(numberOfCells-1, 1);
   uOptNew = ones(numberOfCells-1, 1);
   J = zeros(numberOfCells-1);
   
   converged = 0;
   maxIt = 1000;
   it = 0;

   % 2nd approach stuff
    for i = 1:numberOfCells
        if all(G(i,:) < 10e-2)
            terminalStateID = i;
        end
    end
    G(G==Inf) = 10e5; 
    cellRange = 1:numberOfCells;
    cellRange(terminalStateID) = [];
   
   while ~converged
       it = it + 1;

      % Evaluation (2nd approach);
      for i = 1:(numberOfCells-1)
          evali = cellRange(i);
          Peval(i,:) = reshape(squeeze(P(evali, cellRange, uOpt(i))), 1, numberOfCells-1);
          Geval(i,:) = G(evali, uOpt(i));
      end

      J = (eye(numberOfCells-1) - Peval)\Geval;

      % Evaluation
      %for i=1:numberOfCells
      %    parSum = zeros(numberOfCells, 1);
      %    for j=1:numberOfCells
      %          parSum(i) = P(i, j, uOpt(i)).*J(j) + parSum(i);
      %    end

      %    J(i) = G(i, uOpt(i)) + parSum(i);
      %end
      
      % Improvement
      %for i=1:numberOfCells
      %    parSum2 = zeros(numberOfCells, numberOfInputs);
      %    for j=1:numberOfCells
      %        parSum2(i,:) = squeeze(P(i, j, controlSpaceID))'.*J(j) + parSum2(i,:);
      %    end
          
      %    [JUpdate(i), uOpt(i)] = min(G(i, controlSpaceID) + parSum2(i,:));
          
      %end

      % Improvement (2nd approach)
      Pimp = P;
      Pimp(terminalStateID, :, :) = [];
      Pimp(:, terminalStateID, :) = [];
      Gimp = G;
      Gimp(terminalStateID, :) = [];

      for u = 1:numberOfInputs
          Psliced(:,u) = squeeze(Pimp(:,:, u))*J;
      end

      [~, uOptNew] = min(Gimp + Psliced, [], 2);
      
      if (abs(max(uOptNew - uOpt)) < 1e-50)
          converged = 1;
      end

      uOpt = uOptNew;

   end
   
   J = [J(1:(terminalStateID-1)); 0; J(terminalStateID:end)];
   J_opt = J';
   uOpt = [uOpt(1:(terminalStateID-1)); 1; uOpt(terminalStateID:end)];
   u_opt_ind = uOpt';
   
   fprintf('Policy iteration terminated after %d iterations', it);
      
end

