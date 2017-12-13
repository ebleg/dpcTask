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
   
   uOpt = ones(numberOfCells, 1);
   J = zeros(numberOfCells);
   
   converged = 0;
   maxIt = 1000;
   it = 0;
   
   while ~converged
       it = it + 1;
      % Evaluation
      for i=1:numberOfCells
          parSum = zeros(numberOfCells, 1);
          for j=1:numberOfCells
            parSum(i) = P(i, j, uOpt(i)).*J(j) + parSum(i);
          end

          J(i) = G(i, uOpt(i)) + parSum(i);
      end
      
      % Improvement
      for i=1:numberOfCells
          parSum2 = zeros(numberOfCells, numberOfInputs);
          for j=1:numberOfCells
              parSum2(i,:) = squeeze(P(i, j, controlSpaceID))'.*J(j) + parSum2(i,:);
          end
          
          [JUpdate(i), uOpt(i)] = min(G(i, controlSpaceID) + parSum2(i,:));
          
      end
      
      if (abs(max(max((JUpdate - J))) < 1e-50) || it > maxIt)
          converged = 1;
      end
      
      J = JUpdate;
   end
   
   J_opt = J';
   u_opt_ind = uOpt';
   
   fprintf('Policy iteration terminated after %d iterations', it);
      
end

