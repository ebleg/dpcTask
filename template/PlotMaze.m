function figH = PlotMaze(fig, mazeSize, walls, targetCell, holes, resetCell, varargin)
%PLOTMAZE Can plot a maze, the costs for each cell and the control action
%in each cell.
%   
%       figH = PlotMaze(fig, mazeSize, wall, targetCell, holes, resetCell,
%       varargin)
%
%       Input arguments:
%           fig:
%               Figure handle of plot.
%
%           mazeSize:
%               1x2-vector with first element being the width of the maze,
%               second element being the height of the maze.
%
%           walls:
%               2Kx2-matrix containing the K wall segments, where the start
%               and end point of the k-th segment are stored in row 2k-1
%               and 2k, respectively.
%
%           targetCell:
%               1x2-vector describing the position of the target cell in
%               the maze.
%
%           holes:
%               Hx2-matrix containg the H holes of the maze. Each column
%               represents the position of a hole.
%
%           resetCell:
%               1x2-vector describing the position of the reset cell in the
%               maze.
%
%           varargin (optional):
%               Input argument list:
%                   1:  MNx2-matrix 'stateSpace', where each row represents
%                       an element of the state space. Note that the state
%                       space also contains the target cell, in order to
%                       simplify state indexing.
%                   2:  Lx2-matrix 'controlSpace', where the l-th row 
%                       represents the l-th element of the control space.
%                   3:  MNx1-matrix 'J' containing the optimal cost-to-go
%                       for each element of the state space.
%                   4:  MNx1 matrix containing the indices of the optimal
%                       control inputs for each element of the state space.
%
%       Output arguments:
%           figH:
%               Figure handle to generated plot.



figH = figure(fig);

%% color cells according to costs
cMap = jet(101);
cScale = (0:0.01:1)';

if nargin > 9
    x = varargin{ 1 };
    J = varargin{ 3 };
    
    maxJ = max(J);
    
    for i = 1 : length( J )
        xCorner = [x(i,1)-1 x(i,1) x(i,1) x(i,1)-1];
        yCorner = [x(i,2)-1 x(i,2)-1 x(i,2) x(i,2)];
        cHSV = rgb2hsv(interp1(cScale, cMap, J(i)/maxJ));
        cHSV(2) = 0.5*cHSV(2);
        fill(xCorner, yCorner, hsv2rgb(cHSV), 'EdgeColor', 'none');
        hold on;
    end
end


%% plot holes
for i = 1:1:size(holes,1)
    c = holes(i,:) - 0.5;
    r = 0.25;
    pos = [c-r, 2*r, 2*r];
    rectangle('Position', pos, 'Curvature', [1,1], 'FaceColor', 'black', 'Edgecolor', 'none');
    hold on;
end

%% plot reset cell
if ~isempty(resetCell)
    c = resetCell - 0.5;
    r = 0.25;
    pos = [c-r, 2*r, 2*r];
    rectangle('Position', pos, 'Curvature', [0 0], 'FaceColor', 'blue', 'Edgecolor', 'none');
    hold on;
end

%% plot target cell
c = targetCell - 0.5;
r = 0.25;
pos = [c-r, 2*r, 2*r];
rectangle('Position', pos, 'Curvature', [0 0], 'FaceColor', 'red', 'Edgecolor', 'none');
hold on;

%% plot (outer) boundaries
plot([0 mazeSize(1)], [0 0], 'k', 'LineWidth', 4);
hold on;
plot([mazeSize(1) mazeSize(1)], [0 mazeSize(2)], 'k', 'LineWidth', 3);
plot([mazeSize(1) 0], [mazeSize(2) mazeSize(2)], 'k', 'LineWidth', 3);
plot([0 0], [mazeSize(2) 0], 'k', 'LineWidth', 4);


%% plot (inner) walls
if ~isempty(walls)
    for i = 1:(size(walls,1)/2)
        plot(walls(2*i-1:2*i,1), walls(2*i-1:2*i,2), 'k', 'LineWidth', 3);
    end
end

%% plot cells
for i=1:mazeSize(1)
    for j=1:mazeSize(2)
        plot([0 mazeSize(1)], [j j], 'k', 'LineWidth', 0.5, 'LineStyle', ':');
        plot([i i], [0 mazeSize(2)], 'k', 'LineWidth', 0.5, 'LineStyle', ':');
    end
end

%% plot arrows and expected costs
if nargin > 9
    x = varargin{ 1 };
    u = varargin{ 2 };
    J = varargin{ 3 };
    u_opt_ind = varargin{ 4 };
    
    for i = 1 : length( J )
        x_i = x( i, : );
        shouldPlotArrow = true;
        shouldPlotCost = true;
        if isequal( x_i, targetCell' )
            shouldPlotArrow = false;
            shouldPlotCost = false;
        end
        
        if shouldPlotCost
            text( x_i( 1 ) - 0.9, x_i( 2 ) - 0.1, num2str( J( i ), 3 ), 'FontSize', 8 );
        end
        if shouldPlotArrow
            center = [ x_i( 1 ) - 0.5, x_i( 2 ) - 0.5 ];
            u_i = u( u_opt_ind( i ), : );
            
            if sum( abs( u_i ) ) > 0
                startPt = center + 0.2 * u_i / norm( u_i );
                endPt = center + 0.5 * u_i / norm( u_i );
                steps = max(abs(u_i));
                arrow( startPt, endPt, steps );
            end
        end
    end
end

%% label plot and correct aspect ratio
title(strcat('Maze (width=', num2str(mazeSize(1)), ', height=', num2str(mazeSize(2)), ')'));
daspect([1 1 1]);
margin = 2;
axis([-margin mazeSize(1)+margin -margin mazeSize(2)+margin]);

findfigs;
hold off;

end

function arrow( startPt, endPt, scale )

% compute orientation if arrow head
alpha = atan2( endPt( 2 ) - startPt( 2 ), endPt( 1 ) - startPt( 1 ) );
R = [ cos( alpha ), -sin( alpha ); sin( alpha ), cos( alpha ) ];

% define lines that draw the arrow head
arrowHead = [ 0, 0;
              -0.1, 0.1;
              0, 0;
              -0.1, -0.1 ]*scale;
for i = 1 : size( arrowHead, 1 )
    arrowHead( i, : ) = ( R * arrowHead( i, : )' )';
    arrowHead( i, : ) = arrowHead( i, : ) + endPt;
end

% define line that draws the arrow
arrowLines = [ startPt( 1 ), startPt( 2 );
               endPt( 1 ), endPt( 2 ) ];

% plot
color = [ 0, 0, 0 ] + 0.5;
plot( arrowLines( :, 1 ), arrowLines( :, 2 ), 'color', color, 'lineWidth', 1.5 );
plot( arrowHead( :, 1 ), arrowHead( :, 2 ), 'color', color, 'lineWidth', 1.5 );

end

