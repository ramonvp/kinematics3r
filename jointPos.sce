
// Forward kinematics function
function [x,y,gamma]=jointPosition2EndEffectorPose(theta1, theta2, theta3)
    gamma = theta1+theta2+theta3
    x = L1*cos(theta1)+L2*cos(theta1+theta2)+(g1+g3)*cos(gamma) + sin(gamma)*g2
    y = -L1*sin(theta1)+L2*sin(theta1+theta2)+(g1+g3)*sin(gamma) - g2*cos(gamma)
endfunction

// Create a rotation matrix with <angle> radians
function [m]=R(angle)
    m = [cos(angle) -sin(angle); sin(angle) cos(angle)]
endfunction

// Ensure angle is between [-PI, +PI]
function [a] = normalize_angle(angle)
    // First ensure angle is between [-2*PI, +2*PI]
    while angle > 2*%pi
        angle = angle - 2*%pi
    end
    while angle < -2*%pi
        angle = angle + 2*%pi
    end
    
    // Finally get the elbow down angle
    if(angle > %pi)
        angle = 2*%pi - angle
    end
    if(angle < -%pi)
        angle = 2*%pi + angle
    end
    a = angle;
endfunction

// Inverse kinematics function
function [angles]=jointAnglesFromPose(x, y, gamma)
    a12 = L1
    a23 = L2
    p = [x,y]'
    pp = p - R(gamma)*[0, -g2]'
    a34 = g1+g3
    d = 2*a12*a23
    xp = pp(1)
    yp = pp(2)
    f = (xp - a34*cos(gamma))^2 + (yp-a34*sin(gamma))^2 - a12^2 - a23^2
    t2 = acos(f/d)
    if t2 > %pi then
        printf("t2 mas grande que pi\n")
        t2 = 2*%pi - t2
    end
    theta2 = t2

    A = a12 + a23*cos(theta2)
    B = a23*sin(theta2)
    E = xp - a34*cos(gamma)
    F = yp - a34*sin(gamma)

    M = [A -B; B A];
    sol1 = inv(M)*[E F]'
    t1 = atan(sol1(2), sol1(1))

    if t1 > %pi then
        t1 = 2*%pi - t1
    end
    theta1 = t1

    theta3 = normalize_angle(gamma - theta1 - theta2);
    
    angles = [theta1,theta2,theta3];
endfunction

// Forward kinematics function
function [x, y, gamma]=jointPosition2EndEffectorPoseMatrix(theta1, theta2, theta3)
    l1 = [L1;0];
    l2 = [L2;0];
    l3 = [g1+g3;-g2];
    output = R(theta1)*l1 + R(theta1+theta2)*l2 + R(theta1+theta2+theta3)*l3;
    gamma = theta1+theta2+theta3;
    x = output(1);
    y = output(2);
endfunction

function [p]=joint1Position(theta1)
    p = [L1*cos(theta1), L1*sin(theta1)]'
endfunction

function [p]=joint2Position(theta1, theta2)
    pos1 = joint1Position(theta1)
    p = pos1 + R(theta1)*[L2*cos(theta2), L2*sin(theta2)]'
endfunction

function J = jacobian(angles)
    pos1 = joint1Position(angles(1))
    pos2 = joint2Position(angles(1), angles(2))
    J = [0 pos1(2) pos2(2); 0 -pos1(1) -pos2(1); 1 1 1]
endfunction

function valid = isEndPoseValid(pose)
    max_R = L1+L2;
    min_R = abs(L1-L2);
    distance = sqrt(pose(1)*pose(1) + pose(2)*pose(2))
    valid = distance >= min_R & distance <= max_R;
endfunction
