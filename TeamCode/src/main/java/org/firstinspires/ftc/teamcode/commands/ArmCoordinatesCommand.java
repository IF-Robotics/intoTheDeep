package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

public class ArmCoordinatesCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double x;
    private double y;

    public ArmCoordinatesCommand(ArmSubsystem armSubsystem, double x, double y) {
        this.armSubsystem = armSubsystem;
        this.x = x;
        this.y = y;

        addRequirements(armSubsystem);
    }


    @Override
    public void initialize(){
        armSubsystem.setArmCoordinates(x, y);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
