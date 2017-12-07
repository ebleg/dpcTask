function G = ComputeStageCosts( stateSpace, controlSpace, mazeSize, walls, targetCell, holes, resetCell, p_f, c_p, c_r )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, disturbanceSpace,
%   mazeSize, walls, targetCell) computes the stage costs for all states in
%   the state space for all attainable control inputs.
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
%       c_p:
%           Every time the ball bounces into a wall or boundary, we get this number 
%            of time steps as penalty.
%       c_r:
%           Every time the ball falls into a hole, the ball is set to the reset cell
%           at the beginning of the next stage and we get this number of time steps
%           as additional penalty.
%
%   Output arguments:
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.

% put your code here

    M = mazeSize(2); % maze height
    N = mazeSize(1); % maze width
    numberOfCells = M*N; % obvious variable to improve code readability
    numberOfInputs = size(controlSpace, 1); % number of theoretically possible control inputs

    terminalStateID = (targetCell(1)-1)*M + targetCell(2); % what the TA's call target cell
    
    disturbanceSpace = [];
    for i=-1:1 
        for j=-1:1
            disturbanceSpace(end+1, :) = [i, j];
        end
    end
    p_d = 1/size(disturbanceSpace, 1); 

    for cell = 1:numberOfCells 
        % Determine n and k for this cell (CHECK!)

        n = ceil(cell/M); % ceil(.): round to the next integer (1-based indexing)
        m = cell - (n-1)*M; % the only reason why this is complicated is because 1-based indexing is stupid

        cellCenter = [n m] - 0.5;
        allowedControls = controlSpace; % start with empty 'allowedControls'

        %% WALL CHECKING FOR ALLOWED CONTROLS
        for wallID = 1:2:size(walls, 1) % Loop through all the walls       
            wallInit = walls(wallID, :); % First wall corner 
            wallEnd = walls(wallID+1, :);  % Second wall corner
            wallCenter = wallInit + 0.5*(wallEnd - wallInit); % Center of the wall
            wallCorners = [wallInit; wallEnd]; % Vector with the two corners

            % The algorithm evaluates each input step by step (steps of half a cell) and checks for each step whether a 
            % collison with a wall happens. 
            for uID = 1:numberOfInputs % loop through all the inputs
                u = controlSpace(uID, :); % u is the current input we're testing
                inputStep = sign(u).*min(0.5, abs(u)); % Defines the intermediate steps for the input evaluation
                                                       % Input is evaluated with steps of 0.5 or -0.5 in case the input
                                                       % the input is negative. 
                                                       % e.g. u = [-1 0] --> inputStep = [-0.5 0]
                                                       % e.g. u = [0 2] --> inputStep = [0 0.5]
                                                       % e.g. u = [-2 2] --> inputStep = [-0.5 0.5]
                                                       % e.g. u = [0 0] --> inputStep = [0 0]
                collision = 0; % 1 if a collison happend
                evalInput = inputStep; % evalInput = intermediate input to be tested
                inputDone = 0;  % did we evaluate the full input?
                while ~inputDone && ~collision  
                    evalInputIntersect = cellCenter + evalInput;
                    if evalInputIntersect == wallCenter ... % collision with wall center (straight movement)
                      | ismember(evalInputIntersect, wallCorners, 'rows') ... % collision with corners (diagonal movement)
                      | any(evalInputIntersect == 0) ... % boundaries left and bottom
                      | any(evalInputIntersect == [N, M]) % boundaries right and top

                        allowedControls(allowedControls(:,1) == u(1) & allowedControls(:,2) == u(2), :) = []; 

                        % SET G = Inf FOR INFEASIBLE MOVES #########################################################
                        % ##########################################################################################
                            G(cell, uID) = Inf;
                        % ##########################################################################################
                        % ##########################################################################################
                        
                            % remove control input from allowedControls space
                        collision = 1; % we don't have to evaluate the current input anymore, it already failed
                    end

                    if evalInput == u % done!
                        inputDone = 1;
                    end
                    evalInput = evalInput + inputStep; % increment evalInput to move further
                end
            end
        end % end of for walls

        % Wall checking done: determined allowed controls for the current cell
        % ----------------------------------------------------------------------------------------------------------------

        % 'Execute' control policy
        for uID = 1:size(allowedControls, 1)

            % RESET PROBABILITIES FOR EVERY NEW CONTROL LOOP #######################################################
            % ######################################################################################################
                probReset= 0; 
                probBounce = 0;
            % ######################################################################################################
            % ######################################################################################################

            u = allowedControls(uID, :);
            controlSpaceID = find(controlSpace(:,1)==u(1) & controlSpace(:,2)==u(2));
            target = [n m] + u;
            targetCenter = target - 0.5;
            holeFactor = 1;

            %% HOLE CHECKING 
            inputStep = sign(u).*min(1, abs(u)); % Same as before, now the minimum increment is 1

            % cellCenter can be reused from the previous algorithm
            targetReached = 0;
            inputEval = inputStep;
            evalInputCell = cellCenter;

            while ~targetReached & any(u ~= [0 0])
                evalInputCell = cellCenter + inputEval; 
                for holeID = 1:size(holes, 1) 
                    hole = holes(holeID, :);
                    holeCenter = hole - 0.5; 
                        
                    % Check if there is a hole in the cell
                    if evalInputCell == holeCenter 
         
                        % RESET PROBABILITIES FOR EVERY NEW CONTROL LOOP ########################################
                        % #######################################################################################
                            probReset = probReset + holeFactor*p_f; 
                        % #######################################################################################
                        % #######################################################################################

                        holeFactor = holeFactor*(1-p_f); % Update holefactor
                    end
                end
                
                % Check if we have reached the target
                if inputEval == u
                    targetReached = 1;
                end
                inputEval = inputEval + inputStep;
            end

        % ----------------------------------------------------------------------------------------------------------------

            for wID = 1:size(disturbanceSpace, 1) % loop over disturbances
                w = disturbanceSpace(wID, :);
                bounce = 0; % has to be updated again for each disturbance!

                % Determine whether the current disturbance results in a bounce
                for wallID = 1:2:size(walls, 1)
                    wallInit = walls(wallID, :);
                    wallEnd = walls(wallID+1, :);
                    wallCenter = wallInit + 0.5*(wallEnd - wallInit);
                    disturbanceCenter = targetCenter + 0.5*w;

                    if disturbanceCenter == wallCenter ... % collision with wall center (straight movement)
                      | ismember(disturbanceCenter, wallCorners, 'rows') ... % collision with corners (diagonal movement)
                      | any(disturbanceCenter == 0) ... % boundaries left and bottom
                      | any(disturbanceCenter == [N, M]) % boundaries right and top
                        
                        bounce = 1;
                    end
                end

                % If there is a bounce, the final cell will be the target cell
                if bounce == 1
                    final = target;

                    % ADD BOUNCE PROB TO PROBBOUNCE #########################################################
                    % #######################################################################################
                        probBounce = probBounce + p_d;
                    % #######################################################################################
                    % #######################################################################################

                elseif bounce == 0;
                    final = target + w;
                end

                % Is there a hole in the final cell
                finalHole = 0; % 0 if no final hole, 1 if there is, initialize with 0
                for holeID = 1:size(holes, 1)
                    hole = holes(holeID, :);
                    if all(hole == final) && any(w ~= [0 0])

                        % INCLUDE FINAL HOLE IN PROBRESET ##################################################
                        % ##################################################################################
                            probReset = probReset + holeFactor*p_d*p_f;
                        % ##################################################################################
                        % ##################################################################################

                    end
                end
            end % end of loop through disturbances

            % AFTER ALL DISTURBANCE ITERATIONS WRITE G ####################################################
            % #############################################################################################
                G(cell, controlSpaceID) = 1 + ... % time penalty
                                          probReset*c_r + ... % reset penalty: both for final hole as holes along the way
                                          holeFactor*probBounce*c_p; % bouncing probability
            % #############################################################################################
            % #############################################################################################

        end % end of loop through allowed policies
    end % end of loop cells

    % OVERWRITE COSTS FOR TERMINAL STATE #################################################################
    % ####################################################################################################
        G(terminalStateID, :) = zeros(1, size(G,2));
    % ####################################################################################################
    % ####################################################################################################

% ------------------------------------- EO EMIEL'S CODE -------------------------------------------------------------
end % end of function
