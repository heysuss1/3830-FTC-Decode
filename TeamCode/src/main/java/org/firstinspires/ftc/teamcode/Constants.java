package org.firstinspires.ftc.teamcode;

public class Constants {
    public enum Team{
        RED,
        BLUE
    }
    static Team TEAM;
    static double Y_GOAL = 140;

    public static Team getTEAM() {
        return TEAM;
    }

    public static void setTeam(Team team){
        TEAM = team;
    }
}
