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
% This function impl ements the decentralized controller that all robots use 
% for clearing waypoint 1.
function [u,saveData]=controller1(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
    r = 15;
    connted = 0;
    
    M = size(nbrData(:,1));
    if(M(1) == 5)
        connted = 1;
    end
    
    if(uid == 1 && connted)
        u = 10*wpData;
    elseif(uid ~= 1)
        M = size(nbrData(:,1));
        M = M(1);
        
        m = 200;
        
        d = sqrt(nbrData(:,1) .^ 2 + nbrData(:,2) .^ 2);
        c = repmat(2 * r,M,1);
        E0 = ones(size(nbrData(:,1:2)));
        E1 = 20 * (d - 7*r) ./ (d);
        %u = sum(repmat(E4,1,2) .* nbrData(:,1:2)');
        for ii = 1:M
            u = u + E1(ii) * nbrData(ii,1:2)';
        end
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
% @return guardCleared The value 1 if the user-defined guard (evaluated bys
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

function E_d = dist_energy(nbrData,dist_multi)
    Ms = size(nbrData);
    r = 15;
    
    for ii = 1:Ms(1)
        dist(ii) = norm(nbrData(ii,1:2));
    end
    if Ms(1) ~= 0
        E_d = 20 * (dist - dist_multi*r) ./ dist;
    else
        E_d = 1;
    end
end

function a = proj_W_O(w,o)
    w0 = abs(dot(w,o)) * o ./ norm(o);
    a = w - w0';
end

function [safeVect,gain] = closest_obst(obD)
    sensAngl = (2*pi*(0:15)./16);
    
    %Find the closest obstacle vector and only use that
    [O_dist,O_angle] = min(obD);
    
    %Find the weighted average of the obstacle vectors
    
    
    if O_dist < 0.5

        O_vect = [O_dist .* cos(sensAngl(O_angle)), O_dist .* sin(sensAngl(O_angle))]';

        safeVect = -O_vect;
        gain = 500 ./ norm(safeVect);
    else
        safeVect = [1;1];
        gain = 0;
    end
end

function [O_dir,gain] = all_obst(obD)
    sensAngl = (2*pi*(0:15)./16);
    
    %Find the weighted average of the obstacle vectors
    for ii = 1:16
        if isfinite(obD(ii))
            O_dirs(ii,:) = (1/obD(ii)) * [cos(sensAngl(ii)),sin(sensAngl(ii))];
        else
            O_dirs(ii,:) = [0,0];
        end
    end
    O_dir = -mean(O_dirs,1)';
    if norm(O_dir) ~= 0
        O_dir = O_dir / norm(O_dir);
    end
    
    gain = 1;
end

function [u,saveData]=controller2(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0];Ms = size(nbrData);
    nu = [0;0];
    p = 0.44;g = 500;
    
    AA_E = dist_energy(nbrData,9);
    
    for ii = 1:Ms(1)
        distData(ii,1) = norm(nbrData(ii,1:2));
        distData(ii,2) = nbrData(ii,3);
    end
        
    %Compute obstacle energies    
    %[O_V, O_gain] = closest_obst(obstacleData./delta);
    
    [O_V] = all_obst(obstacleData);
    
    
    if uid ~= 1

        %Relax energy/distance requirements
        G = sortrows([nbrData,distData],4);
                
        if nbrData(1,3) == 1
            AA_E(1) = 2 * AA_E(1);
        end
        
        %Make the consensus vector
        for ii = 1:Ms(1)
            nu = nu + AA_E(ii) * nbrData(ii,1:2)';
        end

        p = 0.5;
        res_vect = (p * O_V + (1-p) * nu/norm(nu))
        u = g * res_vect ./ norm(res_vect);
        
    else
        
        res_vect = (p * O_V + (1-p) * wpData/norm(wpData))
        u = g * res_vect ./ norm(res_vect);
    end
    disp(['Agent done: ' num2str(uid) ' with norm u as ' num2str(norm(u))]);
end

function [u,saveData]=controller2alt(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0];
    Ms = size(nbrData);

    disp(num2str(uid));
    AA_E = dist_energy(nbrData);
    
    %Energy for number of edges
    if uid == 1 | uid == 2
        e_num_E = 1/(Ms(1) - 1);
    else
        e_num_E = 1/(Ms(1) - 2);
    end
    Ea = 1/100 .* AA_E .* e_num_E;
    
    %Energies for agent to obstacle
    angles = (2 * pi * (0:15) ./ 16)';
    obstacleData = obstacleData ./ delta; %Renormalize
    
    [a,b] = min(obstacleData);
    if a == inf
        a = 1000;
        obst_vect = [0,0];
    elseif a > 0.1
        obst_vect = [0,0];
    else
        
        obst_vect = [cos(angles(b)),sin(angles(b))]; %Unit direction for the obstacle vector
    end
    Eo = 50 * abs(1./(a - 0.5)); %Weights for the obstacle vector
    
    %integrate vectors
    
    
    %Integrate all into gross movement
    o = -obst_vect';
    if uid == 1
        %Integrate waypoint vector with obstacle vector
        w = wpData;
        
        if norm(o) ~= 0
            theta1 = sqrt(dot(w,o).^2) * acos(dot(w,o) / (norm(w) * norm(o))) / 2;
            fin_vect = rotmatrix(-theta1) * w;
        else
            fin_vect = w;
            disp('NO OBSTACLES!')
        end
        u = (600/norm(fin_vect)) .* fin_vect
        
        %u = (500/norm(wpData)) .* wpData - Eo * obst_vect'; %to subtract the obstacle
        
    elseif uid == 2 & nbrData(1,3) == 1
        
        for ii = 1

            if norm(o) ~= 0
                theta1 = sign(-dot(w,o))
            else
                fin_vect = sum(nbrData(:,1:2)',1);
            end
            u = u + (500/norm(nbrData(ii,1:2))) * AA_E(ii) * fin_vect(:,ii) ;
        end
        o = -obst_vect';
        u = u + o;
    end
end

function [u,saveData]=controller2Ideas(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0];
    Ms = size(nbrData);
    r = 15;
    
    for ii = 1:Ms(1)
        dist(ii) = norm(nbrData(ii,1:2));
    end
    dist = dist';
    atoa_E = 50 * (dist - 2*delta);
    
    if uid == 1 | uid == 2
        e_num_E = 1/(Ms(1) - 1);
    else
        e_num_E = 1/(Ms(1) - 2);
    end
    
    for ii = 1:Ms(1)
        vdiff = (uid .* atoa_E(ii)/e_num_E) * nbrData(ii,1:2)';
        u = u + vdiff;
    end
    
    u = 100 * u ./ norm(u);
end

function [u,saveData]=controller2b(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0];
    Ms = size(nbrData);
    r = 15;
    
    %Neighbor edges
    for ii = 1:Ms(1)
        dist(ii) = norm(nbrData(ii,1:2));
    end
    dist = dist';
    atoa_E = 20 * (dist - 6*r) ./ (dist);
    
    
    
    %obstacle energies
    angles = (2 * pi * (0:15)./16)';
    ob_vects = [obstacleData .* cos(angles), obstacleData .* sin(angles)];
    ob_vects(ob_vects == inf | ob_vects == -inf) = 0;
    ob_vects(isnan(ob_vects)) = 0;
    anti_ob = -sum(ob_vects,1);
    ob_move = 1./(norm(anti_ob)) .* anti_ob;
    
    
    
    
    %Add edge energies so that edges are ~delta apart
    if uid ~= 1
        if Ms(1) > 2 & uid ~= 2
            E0 = ones(size(nbrData(:,1:2)));
            E1 = 10 * (dist - 1*delta) ./ (dist);
            
            for ii = 1:Ms(1)
                u = u + E1(ii) * nbrData(ii,1:2)' + 10*rand(2,1);
            end
        elseif uid == 2 & Ms(1) > 1
            for ii = 1:Ms(1)
                u = u - nbrData(ii,1:2)' + 10*rand(2,1);
            end
        end
    end
    
    if uid == 1 & Ms(1) == 1
        u = 1/5 .* wpData;
    elseif uid == 2 & Ms(1) == 1
        u = u + nbrData(1,1:2)';
    else
        
        u = u + nbrData(1,1:2)';
    end
    
    
    
    
    if (Ms(1) == 1 | Ms(1) == 2) & 0
        if uid ~= 1
            for ii = 1:Ms(1)
                %u = u + nbrData(ii,1:2)';
            end
        else
            %u = 1/5 .* wpData;
        end
    end
    
    
    
    %Move away from obstacles
    u = u + anti_ob';
end

function [u,saveData]=controller2KINDAWORKING(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; 
    
    %%
    %First step is for us to expand into a line, without hitting obstacles
    M = size(nbrData);
    d = sqrt(nbrData(:,1) .^ 2 + nbrData(:,2) .^ 2);
    r = 15;
    %%
    
    %Move into a line
    % 1->2->3->4->5->6
    lined = 0;
    if uid == 6 & M(1) > 1
        for ii = 1:M(1)
            u = u - nbrData(ii,1:2)' + 20*rand(2,1);
        end
    elseif uid == 2 & nbrData(1,3) == 1
        u = u - nbrData(1,1:2)';
        lined = 1;
    elseif uid ~= 1 & M(1) > 2
        for ii = 1:M(1)
            u = u - nbrData(ii,1:2)' + 10*rand(2,1);
        end
    elseif uid ~=1 & M(1) == 2
        lined = 1;
    end
    
    %avoid CLOSEST obstacle
    %WORKS ALMOST FOR SOME DAMN REASON!
    angles = 2 * pi * (0:15)./16;
    
    if uid == 1 & M(1) == 1

        [a,b] = min(obstacleData./delta);
        dot_vect = [0,0];

        if a < 0.5
            vect1 = [a .* cos(angles(b)), a .* sin(angles(b))];
            %1000 here seems to work
            %vect = 1000./((norm(vect1))) .* vect1;
            
            dot_vect = norm(wpData) ./ ((norm(vect1))) .* vect1;
        end
        u = wpData - dot_vect';

        if u == [0;0]
            u = 10 * rand(2,1);
        end
    elseif (uid == 2 & M(1) == 1) | (uid ~= 1 & M(1) == 2 & lined == 1)
        if min(nbrData(:,3) == 1)
            u = u + nbrData(1,1:2)';
        else
            E1 = 20 * (d - 7*r) ./ (d);
            [a,b] = min(E1);
            for ii = 1:M(1)
                u = u + E1(ii) * nbrData(ii,1:2)';
            end
        end
    end
    
    if uid ~= 1
        E1 = 20 * (d - 7*r) ./ (d);
        [a,b] = min(E1);
        vect1 = [a .* cos(angles(b)), a .* sin(angles(b))];
        %1000 here seems to work
       
        dot_vect = norm(mean(nbrData(:,1:2),1)) ./ ((norm(vect1))) .* vect1;

        %u = u + dot_vect';
        %OBSTACLE ATTEMPT
        [a,b] = min(obstacleData./delta);
         if a < 0.5
            vect1 = [a .* cos(angles(b)), a .* sin(angles(b))];
            %1000 here seems to work
            
            obst_vect = norm(u) ./ ((norm(vect1))) .* vect1;
         else
             obst_vect = [0,0];
         end
         u = u - obst_vect';
    end
end

function [u,saveData]=controller2WORKINGISH(uid,nbrData,wpData,obstacleData,missionData,saveData,delta,agentRadius,firstCall)
    u = [0;0]; % Give no control for now, 
               % this is for you to implement
               
    %%
    %First step is for us to expand into a line, without hitting obstacles
    M = size(nbrData);
    d = sqrt(nbrData(:,1) .^ 2 + nbrData(:,2) .^ 2);
    r = 15;
    %%
    
        %Move into a line
    % 1->2->3->4->5->6
    lined = 0;
    if uid == 6 & M(1) > 1
        for ii = 1:M(1)
            u = u - nbrData(ii,1:2)' + 20*rand(2,1);
        end
    elseif uid == 2 & nbrData(1,3) == 1
        u = u - nbrData(1,1:2)';
        lined = 1;
    elseif uid ~= 1 & M(1) > 2
        for ii = 1:M(1)
            u = u - nbrData(ii,1:2)' + 10*rand(2,1);
        end
    elseif uid ~=1 & M(1) == 2
        lined = 1;
    end

    
%     
%     if uid == 6
%         for ii = 1:M(1)
%             u = u - nbrData(ii,1:2)';
%         end
%     elseif uid ~= 1
%         find CLOSEST neighbor
%         d = sqrt(nbrData(:,1) .^ 2 + nbrData(:,2) .^ 2);
%         [~,closest_neig] = min(d);
%         for ii = 1:M(1)
%             u = u - nbrData(ii,1:2)';
%         end
%         u = u + 2 * nbrData(closest_neig)';
% 
%     end
%     
%     lined = 0;
%     
%     if M(1) ~= 2
%         for ii = 1:M(1)
%             u = u - nbrData(ii,1:2)';
%         end
%     else
%         lined = 1;
%     end
    
    %avoid CLOSEST obstacle
    %WORKS ALMOST FOR SOME DAMN REASON!
    angles = 2 * pi * (0:15)./16;
    
    if uid == 1 & M(1) == 1

        [a,b] = min(obstacleData./delta);
        dot_vect = [0,0];

        if a < 0.5
            a
            
            vect1 = [a .* cos(angles(b)), a .* sin(angles(b))];
            %1000 here seems to work
            vect = 1000./((norm(vect1))) .* vect1
            wpData

            dot_vect = norm(wpData) ./ ((norm(vect1))) .* vect1;

        end
        u = wpData - dot_vect';

        if u == [0;0]
            u = 100 * rand(2,1);
        end
    elseif (uid == 2 & M(1) == 1) | (uid ~= 1 & M(1) == 2 & lined == 1)
        if min(nbrData(:,3) == 1)
            u = u + nbrData(1,1:2)';
        else
            E1 = 20 * (d - 7*r) ./ (d);
            [a,b] = min(E1);
            for ii = 1:M(1)
                u = u + E1(ii) * nbrData(ii,1:2)';
            end
        end
        

    end
    
    if uid ~= 1
        E1 = 20 * (d - 7*r) ./ (d);
        [a,b] = min(E1);
        vect1 = [a .* cos(angles(b)), a .* sin(angles(b))];
        %1000 here seems to work
        vect = 1000./((norm(vect1))) .* vect1
        wpData

        dot_vect = norm(mean(nbrData(:,1:2),1)) ./ ((norm(vect1))) .* vect1;

        u = u - dot_vect';
    end
    
%     %Focus on moving the leader to the waypoint without hitting
%     obstacles, with average vectors
%almost works
%     if uid == 1
%         angles = (2 * pi * (0:15)./16);
%         ob = obstacleData;
%                 
%         ib = 1./(obstacleData./delta - 0.2 );
%         ib1 = [ib .* cos(angles'), ib .* sin(angles')];
%         ib2 = ib1;
%         ib2(ib2 == inf | ib2 == -inf) = 0;
%         ib2(isnan(ib2)) = 0;
%         
%         ob1 = [ob .* cos(angles'), ob .* sin(angles')];
%         ob2 = ob1;
%         ob2(ob2 == inf | ob2 == -inf) = 0;
%         ob2(isnan(ob2)) = 0;
%         
%         b3 = sum(ib2,1);
%         obstVect = 100 * b3;
%         
%         u = wpData - obstVect';
%     end
    



    

    
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