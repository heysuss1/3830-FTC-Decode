package org.firstinspires.ftc.teamcode;

public class RobotConstants {
    public enum Team{
        RED,
        BLUE,
    }

    public enum SystemState{
        OFF,
        INTAKING,
        OUTTAKING,
        WAITING,
        SPEEDING_UP,
        WAITING_FOR_SHOT,
        SHOOTING,
        SLOWING_DOWN,
    }
    public enum Motif{
        NULL,
        GPP,
        PGP,
        PPG
    }
    static Team TEAM;
    static Motif MOTIF;
    static double Y_GOAL = 140;

    public static Team getTEAM() {
        return TEAM;
    }
    static double cameraHeight = 6.7; //Inches
    static double cameraAngle = 35; //degrees
    public static void setMotif(Motif motif) {
        RobotConstants.MOTIF = motif;
    }

    public static Motif getMotif() {
        return MOTIF;
    }

    public static void setTeam(Team team){
        TEAM = team;
    }

    public static double ballXOffset = 0, ballYOffset = 0;
}
