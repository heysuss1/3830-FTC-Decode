package org.firstinspires.ftc.teamcode.zArchive;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class Alignment {

    static Robot robot = Robot.getInstance();

    /* NOTE TO THE GOAT RITCHEL JOHN-MARVENS MESIDOR:
    if i put a value in three sets of parenthesis for no reason,
    it's a random ah guess value that probably won't work unless
    you change it to something more reasonable.
     */
    double rotationalTolerance = (((2 /*degrees*/)));
    double distanceTolerance = (((0.5 /*inches*/)));

    // thing to move the wheels to align the angle
    public static void alignYaw(double x, double y, double heading, double goal_x, double goal_y) {
        double targetHeading;
        double rotationalTolerance = (((0.03 /*radians*/)));
        double power = 0;
        DriveTrain.turnDirection direction = DriveTrain.turnDirection.CW;

        targetHeading = Math.atan2(goal_y - y, goal_x - x);

        // determine which direction the bot needs to turn
        if (Math.abs(heading - targetHeading) <= rotationalTolerance )  { // I think 2 degrees of freedom is chill, but that can be changed
            power = 0; //dont turn
        } else if (heading > targetHeading) { // THIS IS ASSUMING CCW IS THE POSITIVE HEADING DIRECTION, FLIP THESE IF ITS NOT
            direction = DriveTrain.turnDirection.CW;
            power = 0.01; // trying to make it so that it slows down and doesn't wiggle too much, idfk if that will work tho
        } else if (heading < targetHeading) {
            direction = DriveTrain.turnDirection.CCW;
            power = 0.01;
        }

        robot.driveTrain.turn(direction, power);

        // also, this should hypothetically be the same exact thing we use for the turret,
        // just instead of robot's heading we figure out the turrets heading
    }

    // thing to move the robot to the correct distance
    public static void alignDistance(double x, double y, double goal_x, double goal_y, double targetDistance) {
        double distanceTolerance = (((0.5 /*inches*/)));

        double distance = Math.sqrt(Math.pow(goal_x - x, 2) + Math.pow(goal_y - y, 2));

        double scalar = (((0.1))) * (distance - targetDistance); // you can change that 0.1

        if (Math.abs(distance - targetDistance) > distanceTolerance ) { // 0.5 inch tolerance guess lol
            robot.driveTrain.setPower(scalar, scalar, scalar, scalar); // just moves the robot forward or back depending on which way it needs to
        }
    }

    public static boolean yawAligned(double x, double y, double goal_x, double goal_y, double heading) {
        double rotationalTolerance = (((0.03 /*rad*/)));

        double targetHeading = Math.atan2(goal_y - y, goal_x - x);
        return (Math.abs(heading - targetHeading) <= rotationalTolerance);
    }

    public static boolean distanceAligned(double x, double y, double goal_x, double goal_y, double targetDistance) {
        double distanceTolerance = (((0.5 /*inches*/)));

        double distance = Math.sqrt(Math.pow(goal_x - x, 2) + Math.pow(goal_y - y, 2));
        return (Math.abs(distance - targetDistance) <= distanceTolerance);
    }
    ///  so, i THINK these codes should work if you just have andy align the robot relatively well
    ///  and just bind them to some button like X or smth for aligning. I have booleans to check
    ///  whether or not the things are aligned, so you could put smth like this: (sorry for shitty pseudocode)

//    while X BUTTON PRESSED:
//        if yawAligned:
//            if distanceAligned:
//                shoot balls
//            else:
//                alignDistance
//        else:
//            alignYaw
//

    ///  yaw is first most important thing, then distance, then balls shooting.
    ///  idk just a thought


}
