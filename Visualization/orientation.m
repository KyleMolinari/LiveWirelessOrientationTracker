%% Kyle Molinari, May 10, 2020

%Takes quaternion data From IMU (BNO055) and displays real time absolute 
%orientation in 3D space

clear;
clc;

%modify serial port and baud rate based on bluetooth connection
s = serialport("COM5", 9600);
disp("Ready"); 

freq = 10; %performs best around 10Hz
period = 1/freq;

%number of samples to be plotted
N = 500;
%current sample
i = 0;

%initialize adjustquat to the identity quaternion
adjustquat = [1 0 0 0];

%quaternion representing the default position set when button is pressed
defaultquat = [0.05 0.01 -0.01 1];

%define local x y and z axes
axes = [0 0 -1;-1 0 0; 0 1 0];

%optional - point cloud to be overlayed onto plot. point cloud can be
%imported as .xyz or .txt in the form X Y Z or X Y Z R G B.
%use point clouds with under 15k points to avoid latency
try
    pointcloud = load('bunny.xyz'); %ex: bunny.xyz, cat.xyz, key.xyz, etc
    pointcloud = pointcloud(:,1:3);
catch
    pointcloud = [0 0 0];
end

%adds colour if point cloud data is of the form X Y Z R G B, or defaults to
%black if no colour is specified
try
    colour = pointcloud(:,4:6);
catch
    colour = [0 0 0]; %default colour
end

%define origin point to be used in plot with point cloud
size = size(pointcloud);
origin = zeros(size(1),1);


while(i<N)
    try
        tic
        %increment current sample
        i=i+1;
        
        %read orientation data from serial port
        data = str2double(split(readline(s), ",")');
        
        %Serial port data structure is of the form:
        %w,x,y,z,accelx,accely,accelz,reset
        %orientation is represented by quaternion w,x,y,z
        %linear acceleration is not used in this project
        %reset can be 0 (no button press) or 1 (button press)
        
        %save data into variables
        %take negative of w, x, and y to flip z axis rotation
        quatw = -data(1); 
        quatx = -data(2);
        quaty = -data(3);
        quatz = data(4);
        reset = data(8);
        
        %if reset flag is set, zero out the rotation
        if reset == 1
            %find inverse quaternion of the current orientation
            [winv, xinv, yinv, zinv] = invquat([quatw quatx quaty quatz]);
            %multiply the current quaternion inverse by the default quaternion position
            [w, x, y, z] = quatmult([winv, xinv, yinv, zinv],defaultquat);
            %update the adjust quaternion which "pulls" the current
            %orientation back to the default quaternion
            adjustquat = [w, x, y, z]/norm([w, x, y, z]);
        end
        
        %create rotation matrix corresponding to current orientation
        rotationmatrix = [1-2*(quaty^2+quatz^2), 2*(quatx*quaty-quatw*quatz), 2*(quatw*quaty+quatx*quatz); ...
            2*(quatx*quaty+quatw*quatz), 1-2*(quatx^2+quatz^2), 2*(quaty*quatz-quatw*quatx); ...
            2*(quatx*quatz-quatw*quaty), 2*(quatw*quatx+quaty*quatz), 1-2*(quatx^2+quaty^2)];
        
        %create another rotation matrix to adjust the current orientation
        %based on the adjust quaternion
        adjustrotation = [1-2*(adjustquat(3)^2+adjustquat(4)^2), ...
            2*(adjustquat(2)*adjustquat(3)-adjustquat(1)*adjustquat(4)), ...
            2*(adjustquat(1)*adjustquat(3)+adjustquat(2)*adjustquat(4)); ...
            2*(adjustquat(2)*adjustquat(3)+adjustquat(1)*adjustquat(4)), ...
            1-2*(adjustquat(2)^2+adjustquat(4)^2), ...
            2*(adjustquat(3)*adjustquat(4)-adjustquat(1)*adjustquat(2)); ...
            2*(adjustquat(2)*adjustquat(4)-adjustquat(1)*adjustquat(3)), ...
            2*(adjustquat(1)*adjustquat(2)+adjustquat(3)*adjustquat(4)), ...
            1-2*(adjustquat(2)^2+adjustquat(3)^2)];
        
        %define local cartesian axes by rotating the defined global axes by
        %the rotation matrix and adjust rotation matrix
        cartaxes = axes*rotationmatrix*adjustrotation;
        cartaxes(1,:) = cartaxes(1,:)/norm(cartaxes(1,:));
        cartaxes(2,:) = cartaxes(2,:)/norm(cartaxes(2,:));
        cartaxes(3,:) = cartaxes(3,:)/norm(cartaxes(3,:));
        
        %rotate the point cloud based on the rotation matrix and adjust
        %rotation matrix
        cartcoords = pointcloud*rotationmatrix*adjustrotation;
        
        %plot local coordinates in RGB
        quiver3(0,0,0, cartaxes(1,1), cartaxes(1,2), -cartaxes(1,3), 0.1, 'Color', 'r');
        hold on
        quiver3(0,0,0, cartaxes(2,1), cartaxes(2,2), -cartaxes(2,3), 0.1, 'Color', 'g');
        quiver3(0,0,0, cartaxes(3,1), cartaxes(3,2), -cartaxes(3,3), 0.1, 'Color', 'b');
        
        %plot point cloud data in specified colour
        scatter3(cartcoords(:,1), cartcoords(:,2), -cartcoords(:,3), 1, colour);
        hold off
        
        %figure specifications
        xlabel("X")
        ylabel("Y")
        zlabel("Z")
        word = "Sample " + i + " of " + N;
        title("Position and Orientation: " + word);
        view(10,10)
        pbaspect([1 1 1])
        drawnow
        
        %display time between plot updates. This should roughly be equal to
        %the reciprocal of the frequency. If it is larger, try reducing the
        %number of points in the point cloud
        toc
                
    catch
        %Display Error Message as Figure
        errorfig = figure('Name','Error','NumberTitle','off', 'Color', 'white');
        errorfig.WindowState = 'maximized';
        str = "Error";
        text(0.4,0.5,str, 'FontSize', 20, 'Color', 'red');
        axis off
        error('Error');
    end
end

