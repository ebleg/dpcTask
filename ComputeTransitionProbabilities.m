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
%    controlSpace = [];
%    for i=-1:1
%       for j=-1:1
%           if (j ~= 0 || i ~= 0)
%               controlSpace(end+1, :) = [i, j]; 
%           end
%       end
%    end
%    controlSpace = [controlSpace; 2*controlSpace; [0 0]];

    % Initialize probability matrix
    P = zeros(numberOfCells, numberOfCells, numberOfInputs);
        % P(i, j, u):
        %   i = initial state
        %   j = final state
        %   u = applied control input
        
    for cell = 1:numberOfCells 
        % Determine n and k for this cell (CHECK!)
        n = ceil(cell/M); % ceil(.): round to the next integer (1-based indexing)
        m = cell - (n-1)*M; % the only reason why this is complicated is because 1-based indexing is stupid

        % -------------------------------------------------------------------------
        %% Determine possible control inputs (this is not the most efficient way...)

        allowedControls = controlSpace;
        
        % WARNING: THE FOLLOWING ALGORITHM WORKS BUT IS TERRIBLY COMPLEX
        % (I am developing a better one)
        % Check for boundaries 
        if m <= 2 allowedControls(find(allowedControls(:, 1) < -(m-1)), :) = [];, end
        if M-m <= 2  allowedControls(find(allowedControls(:, 1) > M-m), :) = [];, end
        if n <= 2 allowedControls(find(allowedControls(:, 2) < -(n-1)), :) = [];, end
        if N-n <= 2  allowedControls(find(allowedControls(:, 2) > N-n), :) = [];, end

        % Check for walls
        for wallID = 1:2:size(walls, 1)
            wallInit = walls(wallID, :);
            wallEnd = walls(wallID+1, :);

            % Check for immediate walls (for straight movements)
            if wallInit(1) == n-1 && wallEnd(1) == n && wallEnd(2) == m
                allowedControls(find(allowedControls(:, 1) > 0), :) = [];, end
            if wallInit(1) == n-1 && wallEnd(1) == n && wallEnd(2) == m-1
                allowedControls(find(allowedControls(:, 1) < 0), :) = [];, end
            if wallInit(2) == m-1 && wallEnd(2) == m && wallEnd(1) == n 
                allowedControls(find(allowedControls(:, 2) > 0), :) = [];, end
            if wallInit(2) == m-1 && wallEnd(2) == m && wallEnd(1) == n-1 
                allowedControls(find(allowedControls(:, 2) < 0), :) = [];, end

            % Check for straight walls at a distance
            if wallInit(1) == n-1 && wallEnd(1) == n && wallEnd(2) == m+1
                allowedControls(find((allowedControls(:, 1) > 1) & (allowedControls(:, 2) == 0)), :) = [];, end
            if wallInit(1) == n-1 && wallEnd(1) == n && wallEnd(2) == m-2
                allowedControls(find((allowedControls(:, 1) < -1) & (allowedControls(:, 2) == 0)), :) = [];, end
            if wallInit(2) == m-1 && wallEnd(2) == m && wallEnd(1) == n+1 
                allowedControls(find((allowedControls(:, 2) > 1) & (allowedControls(:, 1) == 0)), :) = [];, end
            if wallInit(2) == m-1 && wallEnd(2) == m && wallEnd(1) == n-2 
                allowedControls(find((allowedControls(:, 2) < -1) & (allowedControls(:, 1) == 0)), :) = [];, end

            % Check for distant wall corners
            if ((wallInit(2) == m+1 && wallInit(1) == n-2) || (wallEnd(2) == m+1 && wallEnd(1) == n-2))
                allowedControls(find((allowedControls(:,1) == 2) & (allowedControls(:,2) == -2)), :) = [];, end
            if ((wallInit(2) == m+1 && wallInit(1) == n+1) || (wallEnd(2) == m+1 && wallEnd(1) == n+1))
                allowedControls(find((allowedControls(:,1) == 2) & (allowedControls(:,2) == 2)), :) = [];, end
            if ((wallInit(2) == m-2 && wallInit(1) == n-2) || (wallEnd(2) == m-2 && wallEnd(1) == n-2))
                allowedControls(find((allowedControls(:,1) == -2) & (allowedControls(:,2) == -2)), :) = [];, end
            if ((wallInit(2) == m-2 && wallInit(1) == n+1) || (wallEnd(2) == m-2 && wallEnd(1) == n+1))
                allowedControls(find((allowedControls(:,1) == -2) & (allowedControls(:,2) == 2)), :) = [];, end
        end % end of for walls
        % determined allowed controls
        % -------------------------------------------------------------------------

        % 'Execute' control policy
        for uID = 1:size(allowedControls, 1)
            u = allowedControls(uID, :);
            target = [m+u(1), n+u(2)];
            mTarget = target(1);
            nTarget = target(2);

            % determine cells in between
            trajectCells = [];
            trajectCells(1, :) = target;
            if u(1) > 1 || u(2) > 1 
                trajectCells(2, :) = [m + floor(mTarget-m)/2, n + floor(nTarget-n)/2];, end
            
            % check for holes along the way
            for holeID = 1:size(holes, 1) 
                holeFactor = 1; % takes into account the future probabilities that assume the ball did not fall 
                                % in the hole
                hole = holes(holeID, :);     
                for trajectCellID  = 1:size(trajectCells, 1) 
                    if hole == trajectCells(trajectCellID, :)
                        % Update P - looks complicated but really isn't
                        P(n*M+m, ... % original cell 
                          resetCell(2)*M + resetCell(1), ... 
                          find(controlSpace(:, 1) == u(1) & controlSpace(:, 2) == u(2))) = ... 
                            P(n*M+m, ...
                              resetCell(2)*M + resetCell(1), ...
                              find(controlSpace(:, 1) == u(1) & controlSpace(:, 2) == u(2))) + p_f;
                        holeFactor = (1-p_f)*holeFactor; 
                    end % end of hole check
                end % end of for through traject
            end % end of for through holes


            
        end % end of of for through allowed policies

    end % end of for cells
   
    

end % end of function

