package org.firstinspires.ftc.teamcode;

public class Constants {
    public enum Team{
        RED,
        BLUE
    }

    public enum Motif{
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

    public static void setMotif(Motif motif) {
        Constants.MOTIF = motif;
    }

    public static Motif getMotif() {
        return MOTIF;
    }

    public static void setTeam(Team team){
        TEAM = team;
    }
}
