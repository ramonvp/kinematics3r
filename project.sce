
//========================================================
// PARAMETERS
//========================================================

// Dimensions of the robot
L1 = 0.62;
L2 = 0.57;
g1 = 0.1;
g2 = 0.2;
g3 = 0.3;

// initial pose of the robot
startPose = [0.9, -0.2, 0];

// target pose of the robot
endPose = [0.9, -0.7, 0];

// applied twist
T =  [0 -0.1 0];

// time increment in secs
deltaT = 0.01;

// max allowed iterations (in case something goes wrong)
MAX_COUNTER = 1000

//========================================================

exec("jointPos.sce");


if isEndPoseValid(startPose) == %F then
    messagebox("Initial pose is out of robot workspace", "Invalid Pose", "error")
    return
end

if isEndPoseValid(endPose) == %F then
    messagebox("Target pose is out of robot workspace", "Invalid Pose", "error")
    return
end

counter = 1

// starting positions
currentPose = startPose

w1 = []
w2 = []
w3 = []
currentTime = 0
t = []
error = [];

// expected duration in seconds, used for showing a nice graphic
diff = sqrt((endPose(1)-startPose(1))^2 + (endPose(2)-startPose(2))^2);
duration = abs(diff/T(2));

// initial sign of the error
errorSign = csgn(endPose(2)-startPose(2));

h = figure();
h.anti_aliasing = "16x";
//messagebox("Target pose is out of robot workspace", "Error", "error", ["Ok"], "modal")


while 1==1
    angles = jointAnglesFromPose(currentPose(1), currentPose(2), currentPose(3))
    J = jacobian(angles)
    w = inv(J)*T'
    
    error(counter) = endPose(2)-currentPose(2);
    w1(counter) = w(1)
    w2(counter) = w(2)
    w3(counter) = w(3)
    t(counter) = currentTime

    // new angles after delta_T
    angles = angles + deltaT*w'
    [currentPose(1), currentPose(2), currentPose(3)]=jointPosition2EndEffectorPoseMatrix(angles(1), angles(2), angles(3))
    counter = counter + 1
    currentTime = currentTime + deltaT

    // condition to stop iterating: error changes of sign
    if csgn(endPose(2)-currentPose(2)) ~= errorSign
       break; 
    end

    // protection in case something goes wrong
    if counter > MAX_COUNTER
        disp("HE PETADO!")
        break
    end

    drawlater();
    clf();
    h.info_message = "Running simulation...";

    // plot the robot arm configuration
    subplot(211)
    // joint positions
    x = [0]; y = [0]; // first joint is always at 0,0
    pos1 = joint1Position(angles(1));
    pos2 = joint2Position(angles(1), angles(2));
    x(2) = pos1(1);
    x(3) = pos2(1);
    y(2) = pos1(2);
    y(3) = pos2(2);
    
    // end effector decoration
    gamma = currentPose(3)
    g1Point = pos2 + g1*[cos(gamma), sin(gamma)]'
    g2Point = g1Point + g2*[cos(gamma-%pi/2), sin(gamma-%pi/2)]'
    g3Point = g2Point + g3*[cos(gamma), sin(gamma)]'

    x2 = [pos2(1), g1Point(1), g2Point(1), g3Point(1)];
    y2 = [pos2(2), g1Point(2), g2Point(2), g3Point(2)];
    plot2d( x2, y2, style=2);
    g = gce();
    g.children.thickness = 4;


    plot2d( x, y, style=2);

    g = gce();
    g.children.mark_mode = "on";
    g.children.mark_style = 9;
    g.children.mark_size = 2;
    g.children.thickness = 4;
    g.children.mark_foreground = 0;


    xtitle ( "Robot configuration" , "x (m)" , "y (m)" );
    //xtitle("Semilogy", "Iterations", "Error");
    //set(gca(),"grid",[-1 1]);

    axes = gca();
    axes.isoview = "on"
    axes.auto_scale = "off"
    axes.data_bounds = [-0.5,-1.0; 1.5, 0.5];

    // plot the angular velocities    
    subplot(212)
    plot(t, w1,'r', t, w2,'g', t, w3,'b');
    xtitle ( "Angular Velocities" , "time (s)" , "w (rad/s)" );
    legend ( "w1" , "w2", "w3" );
    axes = gca();
    axes.auto_scale = "off"
    axes.data_bounds = [0, -0.5; duration, 0.5];
    drawnow();

    
    sleep(deltaT*1000);

end

// add last calculated pose before breaking the loop
error(counter) = endPose(2)-currentPose(2);
w1(counter) = w(1)
w2(counter) = w(2)
w3(counter) = w(3)
t(counter) = currentTime

plot(t, w1,'r', t, w2,'g', t, w3,'b');
xtitle ( "Angular Velocities" , "time (s)" , "w (rad/s)" );
legend ( "w1" , "w2", "w3" );
h.info_message = "Finished";
printf("Num iterations = %d\n", counter)
printf("Error in position = %f\n", endPose(2)-currentPose(2))

