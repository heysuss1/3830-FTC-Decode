package org.firstinspires.ftc.teamcode.autos;

public class CmdFarZoneAuto implements AutoCommands{

    enum AutoState {
        START,
        DRIVE_TO_SHOOTING_SPOT,
        SHOOTING,
        DRIVE_TO_GROUP,
        SLURPING_GROUP,
        DRIVE_TO_PARK, // <-- dunno if we need this but just in case
        DRIVE_TO_GATE, // <--- these are for later when we get more balls than the pre-placed ones
        SLURPING_FROM_GATE,
        STOP
    }

    AutoState state = AutoState.START;
    @Override
    public void buildPaths(){

    }
    @Override
    public void autonomousUpdate(){

    }
    @Override
    public void cancel(){

    }
    @Override
    public String getAutoState(){
        return state.toString();
    }
}
