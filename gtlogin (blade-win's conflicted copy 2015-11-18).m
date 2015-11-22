% Vineet Tiruvadi
% vtiruvadi3
% ECE8823 Final Project
% 2010 by Georgia Institute of Technology. All rights reserved.

%% Initialization

% function [h]=gtlogin()
% @return controllers Cel of 6 handles to the 6 controllers used between 
%                     the waypoints
% @return guards Cel of 6 handles to the 6 guards between the waypoints
%
% This function returns handles to the 6 controllers used for clearing each
% of the 6 waypoints.
%
% DO NOT CHANGE THIS FUNCTION!!!
function [controllers,guards]=gtlogin()
    controllers = {@controller1, @controller2, @controller3, @controller4, @controller5, @controller6};
    guards = {@guard1,@guard2,@guard3,@guard4,@guard5,@guard6};
end

%% Waypoint 1

% function [u,saveData]=controller1(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
% @param uid The unique identifier of the current robot (1 is leader)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector.  For the leader robot, contains the 
%               relative displacement between the leader (uid = 1) and the 
%               current wp.  For follower robots, value is [0; 0].
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param missionData Used to pass misc. data that is specific for the
%                    scenario. See mission documentation for details.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @param firstCall Takes value 1 if it is the first time that the robot is 
%                  executing this controller.  Used for initialization.
% @return u A (2 x 1) vector giving the velocity control of the robot.
% @return saveData The updated storage memory contents for the robot.  Will
%                  be passed to the robot again the next time it executes a
%                  controller.
%
% This function implements the decentralized controller that all robots use 
% for clearing waypoint 1.
function [u,saveData]=controller1(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
    r = 15;
    if(uid == 1)
        u = wpData;
    else
        M = size(nbrData(:,1));
        M = M(1);
        
        m = 200;
        
        d = sqrt(nbrData(:,1) .^ 2 + nbrData(:,2) .^ 2);
        c = repmat(2 * r,M,1);
        
        dev = d - 3*c;
        
        %u = sum(repmat(E4,1,2) .* nbrData(:,1:2)');
        for ii = 1:M
            u = u + nbrData(ii,1:2)';
        end
        u(dev <= 0) = 0;
    end
end

% function [guardCleared]=guard1(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector containing the relative displacement 
%               between the leader and the current wp.  
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @return guardCleared The value 1 if the user-defined guard (evaluated by
%                      the leader robot) is cleared, and 0 otherwise.  
%
% This function implements the guard condition that the leader robot uses  
% for clearing waypoint 1.
function [guardCleared]=guard1(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
    % Set guardCleared to 1 to not impose any additional 
    % conditions on the mode switches
    guardCleared = 1;
end

%% Waypoint 2

% function [u,saveData]=controller2(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
% @param uid The unique identifier of the current robot (1 is leader)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector.  For the leader robot, contains the 
%               relative displacement between the leader (uid = 1) and the 
%               current wp.  For follower robots, value is [0; 0].
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param missionData Used to pass misc. data that is specific for the
%                    scenario. See mission documentation for details.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @param firstCall Takes value 1 if it is the first time that the robot is 
%                  executing this controller.  Used for initialization.
% @return u A (2 x 1) vector giving the velocity control of the robot.
% @return saveData The updated storage memory contents for the robot.  Will
%                  be passed to the robot again the next time it executes a
%                  controller.
%
% This function implements the decentralized controller that all robots use 
% for clearing waypoint 2.
function [u,saveData]=controller2(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
end

% function [guardCleared]=guard2(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector containing the relative displacement 
%               between the leader and the current wp.  
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @return guardCleared The value 1 if the user-defined guard (evaluated by
%                      the leader robot) is cleared, and 0 otherwise.  
%
% This function implements the guard condition that the leader robot uses  
% for clearing waypoint 2.
function [guardCleared]=guard2(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
    % Set guardCleared to 1 to not impose any additional 
    % conditions on the mode switches
    guardCleared = 1;
end

%% Waypoint 3

% function [u,saveData]=controller3(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
% @param uid The unique identifier of the current robot (1 is leader)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector.  For the leader robot, contains the 
%               relative displacement between the leader (uid = 1) and the 
%               current wp.  For follower robots, value is [0; 0].
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param missionData Used to pass misc. data that is specific for the
%                    scenario. See mission documentation for details.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @param firstCall Takes value 1 if it is the first time that the robot is 
%                  executing this controller.  Used for initialization.
% @return u A (2 x 1) vector giving the velocity control of the robot.
% @return saveData The updated storage memory contents for the robot.  Will
%                  be passed to the robot again the next time it executes a
%                  controller.
%
% This function implements the decentralized controller that all robots use 
% for clearing waypoint 3.
function [u,saveData]=controller3(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
end

% function [guardCleared]=guard3(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector containing the relative displacement 
%               between the leader and the current wp.  
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @return guardCleared The value 1 if the user-defined guard (evaluated by
%                      the leader robot) is cleared, and 0 otherwise.  
%
% This function implements the guard condition that the leader robot uses  
% for clearing waypoint 3.
function [guardCleared]=guard3(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
    % Set guardCleared to 1 to not impose any additional 
    % conditions on the mode switches
    guardCleared = 1;
end

%% Waypoint 4

% function [u,saveData]=controller4(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
% @param uid The unique identifier of the current robot (1 is leader)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector.  For the leader robot, contains the 
%               relative displacement between the leader (uid = 1) and the 
%               current wp.  For follower robots, value is [0; 0].
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param missionData Used to pass misc. data that is specific for the
%                    scenario. See mission documentation for details.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @param firstCall Takes value 1 if it is the first time that the robot is 
%                  executing this controller.  Used for initialization.
% @return u A (2 x 1) vector giving the velocity control of the robot.
% @return saveData The updated storage memory contents for the robot.  Will
%                  be passed to the robot again the next time it executes a
%                  controller.
%
% This function implements the decentralized controller that all robots use 
% for clearing waypoint 4.
function [u,saveData]=controller4(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
end

% function [guardCleared]=guard4(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector containing the relative displacement 
%               between the leader and the current wp.  
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @return guardCleared The value 1 if the user-defined guard (evaluated by
%                      the leader robot) is cleared, and 0 otherwise.  
%
% This function implements the guard condition that the leader robot uses  
% for clearing waypoint 4.
function [guardCleared]=guard4(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
    % Set guardCleared to 1 to not impose any additional 
    % conditions on the mode switches
    guardCleared = 1;
end

%% Waypoint 5

% function [u,saveData]=controller5(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
% @param uid The unique identifier of the current robot (1 is leader)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector.  For the leader robot, contains the 
%               relative displacement between the leader (uid = 1) and the 
%               current wp.  For follower robots, value is [0; 0].
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param missionData Used to pass misc. data that is specific for the
%                    scenario. See mission documentation for details.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @param firstCall Takes value 1 if it is the first time that the robot is 
%                  executing this controller.  Used for initialization.
% @return u A (2 x 1) vector giving the velocity control of the robot.
% @return saveData The updated storage memory contents for the robot.  Will
%                  be passed to the robot again the next time it executes a
%                  controller.
%
% This function implements the decentralized controller that all robots use 
% for clearing waypoint 5.
function [u,saveData]=controller5(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
end

% function [guardCleared]=guard5(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector containing the relative displacement 
%               between the leader and the current wp.  
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @return guardCleared The value 1 if the user-defined guard (evaluated by
%                      the leader robot) is cleared, and 0 otherwise.  
%
% This function implements the guard condition that the leader robot uses  
% for clearing waypoint 5.
function [guardCleared]=guard5(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
    % Set guardCleared to 1 to not impose any additional 
    % conditions on the mode switches
    guardCleared = 1;
end

%% Waypoint 6

% function [u,saveData]=controller6(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
% @param uid The unique identifier of the current robot (1 is leader)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector.  For the leader robot, contains the 
%               relative displacement between the leader (uid = 1) and the 
%               current wp.  For follower robots, value is [0; 0].
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param missionData Used to pass misc. data that is specific for the
%                    scenario. See mission documentation for details.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @param firstCall Takes value 1 if it is the first time that the robot is 
%                  executing this controller.  Used for initialization.
% @return u A (2 x 1) vector giving the velocity control of the robot.
% @return saveData The updated storage memory contents for the robot.  Will
%                  be passed to the robot again the next time it executes a
%                  controller.
%
% This function implements the decentralized controller that all robots use 
% for clearing waypoint 6.
function [u,saveData]=controller6(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
end

% function [guardCleared]=guard6(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
% @param nbrData A (M x 3) matrix of sensed data from neighbors.
%                Each row contains the relative displacement and UID of a
%                sensed neighbor in the format: [relativeX relativeY uid]
% @param wpData A (2 x 1) vector containing the relative displacement 
%               between the leader and the current wp.  
% @param obstacleData A (16 x 1) vector containing the relative distance 
%                     measurements for obstacles around the robot.
% @param saveData A (10 x 1) vector of storage memory for the robot.  Can
%                 be used to store anything, as long as it fits.
% @param delta The furthest away that the robot can sense anything.
% @param agentRadius The radius of the robot, used for obstacle evasion.
% @return guardCleared The value 1 if the user-defined guard (evaluated by
%                      the leader robot) is cleared, and 0 otherwise.  
%
% This function implements the guard condition that the leader robot uses  
% for clearing waypoint 6.
function [guardCleared]=guard6(nbrData,wpData,obstacleData,saveData,delta,agentRadius)
    % Set guardCleared to 1 to not impose any additional 
    % conditions on the mode switches
    guardCleared = 1;
end