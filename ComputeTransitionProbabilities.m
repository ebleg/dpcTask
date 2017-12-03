function P = ComputeTransitionProbabilities( stateSpace, controlSpace, mazeSize, walls, targetCell, holes, resetCell, p_f )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all attainable control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   disturbanceSpace, mazeSize, walls, targetCell) computes the transition
%   probabilities between all states in the state space for all attainable
%   control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (MN x 2) matrix, where the i-th row represents the i-th
%           element of the state space. Note that the state space also
%           contains the target cell, in order to simplify state indexing.
%
%       controlSpace:
%           A (L x 2) matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       mazeSize:
%           A (1 x 2) matrix containing the width and the height of the
%           maze in number of cells.
%
%   	walls:
%          	A (2K x 2) matrix containing the K wall segments, where the start
%        	and end point of the k-th segment are stored in row 2k-1
%         	and 2k, respectively.
%
%    	targetCell:
%          	A (2 x 1) matrix describing the position of the target cell in
%         	the maze.
%       
%       holes:
%         	A (H x 2) matrix containg the H holes of the maze. Each row
%         	represents the position of a hole.
%
%   	resetCell:
%         	A (1 x 2) matrix describing the position of the reset cell in
%           the maze.
%
%       p_f:
%           The probability of falling into a hole when the ball is
%           traversing through or to a cell with a hole
%
%   Output arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.

% put your code here

% ---------------------------- EMIEL'S CODE ----------------------------------
    M = mazeSize(2); % maze height
    N = mazeSize(1); % maze width
    numberOfCells = M*N; % obvious variable to improve code readability
    numberOfInputs = 17; % number of theoretically possible control inputs
    
    % Establish control space 
    controlSpace = [];
    for i=-1:1
       for j=-1:1
           if (j ~= 0 || i ~= 0)
               controlSpace(end+1, :) = [i, j]; 
           end
       end
    end
    controlSpace = [controlSpace; 2*controlSpace; [0 0]];

    % Initialize probability matrix
    P = zeros(numberOfCells, numberOfCells, numberOfInputs);
        % P(i, j, u):
        %   i = initial state
        %   j = final state
        %   u = applied control input
        
    for cell = 1:numberOfCells 
        % Determine possible control inputs
        allowedControls = controlSpace;
        for u=1:numberOfInputs
            if u(1) == 0 || u(2) == 0 % straight movement

            end
              
        end
    end
   
    

end

