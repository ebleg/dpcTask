
mazeSize = [ 6, 5 ]; % N, M
	[ walls, targetCell, holes, resetCell ] = GenerateMaze( mazeSize( 1 ), ...
        mazeSize( 2 ), true );
    

clear walls;
targetCell
resetCell
holes
walls(1,:)=[resetCell(1)-1 resetCell(2)-1];
walls(2,:)=[resetCell(1)-1 resetCell(2)-0];
walls(3,:)=[resetCell(1)-1 resetCell(2)-0];
walls(4,:)=[resetCell(1)-0 resetCell(2)-0];
walls(5,:)=[resetCell(1)-0 resetCell(2)-0];
walls(6,:)=[resetCell(1)-0 resetCell(2)-1];
walls(7,:)=[resetCell(1)-1 resetCell(2)-1];
walls(8,:)=[resetCell(1)-0 resetCell(2)-1];

walls(9,:)=[targetCell(1)-1 targetCell(2)-1];
walls(10,:)=[targetCell(1)-1 targetCell(2)-0];
walls(11,:)=[targetCell(1)-1 targetCell(2)-0];
walls(12,:)=[targetCell(1)-0 targetCell(2)-0];
walls(13,:)=[targetCell(1)-0 targetCell(2)-0];
walls(14,:)=[targetCell(1)-0 targetCell(2)-1];
walls(15,:)=[targetCell(1)-1 targetCell(2)-1];
walls(16,:)=[targetCell(1)-0 targetCell(2)-1];

walls

save MazeExample walls targetCell holes resetCell mazeSize;