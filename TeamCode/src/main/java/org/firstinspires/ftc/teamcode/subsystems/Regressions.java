package org.firstinspires.ftc.teamcode.subsystems;


// shooting position: 131 137 red, 13 137 blue
public class Regressions {
    public ShotLocation[] lookUpTable = {
            new ShotLocation(30, 100)
    };

    public int getClosestRPM(double distance){
        int closestRPM = lookUpTable[0].rpm;
        double closestDistance = Math.abs(lookUpTable[0].distance - distance);
        for (int i = 0; i < lookUpTable.length; i++){
            double shotDistance =  Math.abs(distance - lookUpTable[i].distance);
            if (shotDistance < closestDistance){
                closestDistance = shotDistance;
                closestRPM = lookUpTable[i].rpm;
            }
        }
        return closestRPM;
    }

    /*
   This method will determine the pitch needed to make it into the goal based on the distance and rpm.
    * this will be used as well after each shot to determine based on the current drop in velocity, how much does
    the pitch need to adjust.
    */
    public static double getPitch(double distance, int rpm) { //get these (((n))) thru regressions
        // one big happy regression
        return 6.7;
    }




}
