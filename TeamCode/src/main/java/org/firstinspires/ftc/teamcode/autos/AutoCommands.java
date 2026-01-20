package org.firstinspires.ftc.teamcode.autos;

public interface AutoCommands
{
    void autonomousUpdate();

    void buildPaths();

    void startAutoCommand();

    void cancel();

    String getAutoState();
}
