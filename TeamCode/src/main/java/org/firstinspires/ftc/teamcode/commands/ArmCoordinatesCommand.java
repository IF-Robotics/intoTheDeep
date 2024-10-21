package org.firstinspires.ftc.teamcode.commands;

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
    public void execute(){
        armSubsystem.setArmCoordinates(x, y);
    }
}
