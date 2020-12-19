grid = rosmessage( 'nav_msgs/OccupancyGrid');
sub = rossubscriber('/move_base/local_costmap/costmap');

grid = receive(sub,30);

a = grid.Data;
sizex = grid.Info.Width;
sizey = grid.Info.Height;

a = reshape(a, sizex, sizey);
imshow(a,'InitialMagnification', 400);



