package org.firstinspires.ftc.teamcode;

public class Constants {
    public enum Team{
        RED,
        BLUE
    }
    static Team TEAM;
    static double Y_GOAL = 140;

    public static void setTeam(Team team){
        TEAM = team;
    }
}
