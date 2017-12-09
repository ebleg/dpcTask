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
    
    %numberOfInputs = size(G, 2);   
    %numberOfCells = size(G, 1);

    %f = -ones(numberOfCells, 1);

    %b = [];
    %A = [];

    %for u = 1:numberOfInputs
    %    for i = 1:numberOfCells
    %        if G(i, u) ~= Inf
    %            b = [b; G(i, u)];
    %            vec = zeros(1, numberOfCells);
    %            vec(i) = 1;
    %            A = [A; vec - squeeze(P(i,:,u))];
    %        end
    %    end
    %end
      
    %x = linprog(f, A, b);
    %x
    
    %% TWANS CODE
    %% init
    % because 0*inf = NaN, replace all the infinite costs with 1. Doesn't
    % matter because
    G(G==inf) = 1e3;

    % Extract number of states and control inputs
    [K,L] = size(G);

    % Construct linear programming matrices

    % Cost function
    f = -ones(K,1);

    % Inequality constraints 
    b = reshape( G' , K * L , 1 ); 
    A = zeros( K * L , K );

    for i = 1:K
        for l = 1:L
            A(L*(i-1)+l,:) = -P(i,1:K,l);
            A(L*(i-1)+l,i) = 1 + A(L*(i-1)+l,i);
        end
    end

    %% Solve linear program
    J_opt = linprog(f,A,b);

    %% Extract optimal control
    u_opt_ind = -ones(K,1);
    tol = 10^-5;
    for i = 1:K
        for l = 1:L
            if norm(A(L*(i-1)+l,:)*J_opt-b(L*(i-1)+l)) < tol % If constraint is active
                u_opt_ind(i) = l;
                break; % done, go look for next state 
            end
            
            if l == L
                error('Something went wrong when extracting optimal control from linprog solution!'); 
            end
        end
    end
        
end

