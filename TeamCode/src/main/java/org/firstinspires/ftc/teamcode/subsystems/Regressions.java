package org.firstinspires.ftc.teamcode.subsystems;


// shooting position: 131 137 red, 13 137 blue
public class Regressions {

    public static double getRPM(double distance) { //get these (((n))) thru testing
        if (distance <= 12)    return (((67))); // very uncommon
        if (12 < distance && distance <= 24)   return (((67)));
        if (24 < distance && distance <= 36)   return (((67)));
        if (36 < distance && distance <= 48)   return (((67)));
        if (48 < distance && distance <= 60)   return (((67)));
        if (60 < distance && distance <= 72)   return (((67)));
        if (72 < distance && distance <= 84)   return (((67)));
        if (84 < distance && distance <= 96)   return (((67)));
        if (96 < distance && distance <= 108)  return (((67)));
        if (108 < distance && distance <= 120) return (((67))); // enough for the big shooting zone
        if (120 < distance && distance <= 132) return (((67)));
        if (132 < distance && distance <= 144) return (((67)));
        if (144 < distance && distance <= 156) return (((67))); // far zone territory
        if (156 < distance && distance <= 168) return (((67))); // idk how you're shooting from here but ok
        if (168 < distance)                    return (((67)));
        return 0;
    }

    public static double getPitch(double distance, int rpm) { //get these (((n))) thru regressions
        // one big happy regression
    }




}
