
rosshutdown;
rosinit;

REF_ROBOT='quad_handle';
REF_MAP='map';

MAX_PONTOS=10;
count=1;

pontos = [[ 2 8.8 127 ];[ 3 0.5 127 ];[ 5 9.5 127 ];[ 7 0.5 127 ];[ 9 9.5 127 ];[ 9 0.5 127 ];[ 7 9.5 127 ];[ 4.3 0.5 127 ];[ 3 9.5 127 ];[ 1 1.0 127 ]];

goal = rosmessage('geometry_msgs/PoseStamped');
goal.Header.FrameId = 'map';

pub = rospublisher('/move_base_simple/goal');

%grid = rosmessage( 'nav_msgs/OccupancyGrid');
%sub = rossubscriber('/move_base/global_costmap/costmap');

tftree = rostf;

while count <= MAX_PONTOS    

    tftree.AvailableFrames;    

    goal.Pose.Position.X = pontos(count,1);
    goal.Pose.Position.Y = pontos(count,2);
    goal.Pose.Orientation.W = pontos(count,3);
    send(pub,goal);

    erro = 1;    
    while erro > 0.3
        tftree.AvailableFrames;
        pause(1);

        pos = getTransform(tftree,REF_MAP,REF_ROBOT);
        x = pos.Transform.Translation.X;
        y = pos.Transform.Translation.Y;
        erro = sqrt( ( goal.Pose.Position.X - x )^2  + ( goal.Pose.Position.Y - y )^2 );
        disp([erro pos.Transform.Translation.X pos.Transform.Translation.Y ]);        

    end

    count = count + 1;
    
%     try 
%         grid = receive(sub,45);
%         a = grid.Data;
%         sizex = grid.Info.Width;
%         sizey = grid.Info.Height;
% 
%         a = reshape(a, sizex, sizey);
%         imshow(transpose(a),'InitialMagnification', 200);
%     
%     catch ee
%         disp(ee);
%     end
end





