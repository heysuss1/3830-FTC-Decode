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

    static double RED_X_goal = 12; // figure out where the goal is; whichever goal is the correct one to shoot to
    static double BLUE_X_GOAL = 132;
    static double Y_GOAL = 140.4;
    static double g = 386; //gravitational constant, in/sec^2
    static double timepoly_a = g*g/4;
    static double height = 35; //amount of height that ball needs to cover, in.
    static double v0 = 528; //initial velocity, in/sec (30 mph from avi's estimate)

    //AIM:
    double x_ball; // get x-pos and y-pos from pp + difference btwn pedro bot center and ball position
    double y_ball; // X AND Y POS OF THE BALL

    public double getDistance(double x_ball, double y_ball, Constants.Team team){
        double x_goal = team == Constants.Team.BLUE ? 132: 12;
        return Math.sqrt(Math.pow(y_ball-Y_GOAL,2)+Math.pow(x_ball-x_goal,2));
    }

    public double getYaw(double x_ball, double y_ball, Constants.Team team){
        double x_goal = team == Constants.Team.BLUE ? 132: 12;
        return Math.atan2(y_ball-Y_GOAL, x_ball-x_goal);
    }

//    double distance = Math.sqrt(Math.pow(y_ball-y_goal,2)+Math.pow(x_ball-x_goal,2));

    public double getCannonVelocity(){
        return 6.7; //TODO
    }

    //TODO: come back later
    //Spin of the cannon relative to the ground
//    double yaw = Math.atan((y_ball-Y_GOAL)/(x_ball-x_goal));

    public double getPitch(double x_ball, double y_ball, Constants.Team team){

        //If blue team, goal pos = 132, if red it equals 12
        double x_goal = team == Constants.Team.BLUE ? 132: 12;

        //Calculates the time it takes for ball to reach the depot from the cannon
        double timepoly_b = 2*g*height - Math.pow(v0,2);
        double timepoly_c = Math.pow(getDistance(x_ball, y_ball, team),2) + Math.pow(height,2);

        //TODO: figure out which ones better through testing
        double time1 = Math.sqrt(quadratic_formula(timepoly_a, timepoly_b, timepoly_c, 1));
        double time2 = Math.sqrt(quadratic_formula(timepoly_a, timepoly_b, timepoly_c, -1));
        double time = Math.min(time1, time2);

        // returns the pitch!!!
        return Math.atan2(height + g*Math.pow(time,2)*.5, getDistance(x_ball, y_ball, team));

    }



    //

}
