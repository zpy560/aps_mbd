function [poses,pathCosts,MotionLength] = RSheep(startPose,goalPose)
%Create a reedsSheppConnection object.
reedsConnObj = reedsSheppConnection;
reedsConnObj.MinTurningRadius = 6;
%Define start and goal poses as [x y theta] vectors.

%Calculate a valid path segment to connect the poses.
[pathSegObj,pathCosts] = connect(reedsConnObj,startPose,goalPose);
length = pathSegObj{1}.Length;
poses = interpolate(pathSegObj{1},0:0.1:length);

%add direction information to the poses matrix
MotionLength = pathSegObj{1}.MotionLengths;
Interpolate = pathSegObj{1}.interpolate;
MotionDirections = pathSegObj{1}.MotionDirections;

j = 1;
for i = 1: size(poses,1)
    poses(i,4) = MotionDirections(j);
    if poses(i,1)==Interpolate(j+1,1) && poses(i,2)==Interpolate(j+1,2)
        j = j + 1;
    end
end

%Show the generated path.
plot(poses(:,1),poses(:,2),'Color','b','LineWidth',3);
% show(pathSegObj{1});
hold on;
% legend off;