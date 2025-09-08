package org.firstinspires.ftc.teamcode;

/*
Inputs: initial position of ball (position of goal, constant)
Outputs: theta, psi, v0,
 */
public class kinematics {
    //Asher says make a chart of distance needed to cover vs. v0, psi, probably smart

    public static double quadratic_formula(double a, double b, double c, int sign) {
        // literally the quadratic formula because i need it for the projectile motion calculation lol
        return (-b + sign * Math.sqrt(b*b-4*a*c)) / (2*a);
    }

    static double x_goal = 100; // figure out where the goal is; whichever goal is the correct one to shoot to
    static double y_goal = 100;
    static double g = 386; //gravitational constant, in/sec^2
    static double timepoly_a = g*g/4;
    static double height = 35; //amount of height that ball needs to cover, in.
    static double v0 = 528; //initial velocity, in/sec (30 mph from avi's estimate)

    //AIM:
    double x_ball = 0; // get x-pos and y-pos from pp + difference btwn pedro bot center and ball position
    double y_ball = 0; // X AND Y POS OF THE BALL

    double distance = Math.sqrt(Math.pow(y_ball-y_goal,2)+Math.pow(x_ball-x_goal,2));
    double timepoly_b = 2*g*height - Math.pow(v0,2);
    double timepoly_c = Math.pow(distance,2) + Math.pow(height,2);

    double yaw = Math.atan((y_ball-y_goal)/(x_ball-x_goal));

    double time1 = Math.sqrt(quadratic_formula(timepoly_a, timepoly_b, timepoly_c, 1));
    double time2 = Math.sqrt(quadratic_formula(timepoly_a, timepoly_b, timepoly_c, -1));
    double time = Math.max(time1, time2);

    double pitch = Math.atan2(height + g*Math.pow(time,2)*.5, distance);


    //

}
