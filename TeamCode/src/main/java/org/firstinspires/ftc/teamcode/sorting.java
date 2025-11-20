package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class sorting {
    String[] motifs = {"GPP", "PGP", "PPG"};

    // high shot = 6767 ms
    // low shot = 4141 ms
    public int[] getInstructions(String storageMotif, String targetMotif) {
        // switch on the concatenation of storageMotif and targetMotif
        int[] instruction = new int[5];
        // default wait time
        int D = 500;
        switch (storageMotif + " " + targetMotif) {
            case "GPP GPP": case "PGP PGP": case "PPG PPG":
                instruction = new int[] {4141, D, 4141, D, 4141}; //A
                break;
            case "GPP PGP":
                instruction = new int[] {6767, D, 4141, 1000, 4141};
                break;
            case "GPP PPG":
                instruction = new int[] {6767, D, 4141, D, 4141}; //B
                break;
            case "PGP GPP":
                instruction = new int[] {6767, D, 4141, D, 4141}; //C
                break;
            case "PGP PPG":
                instruction = new int[] {4141, D, 6767, D, 4141}; //C
                break;
            case "PPG GPP":
                instruction = new int[] {6767, D, 6767, D, 4141};
                break;
            case "PPG PGP":
                instruction = new int[] {4141, D, 6767, D, 4141}; //B
                break;
        }

        return instruction;
    }

}

