package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

import java.util.function.Supplier;

public class ArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double targetAngle;

    public ArmCommand(ArmSubsystem armSubsystem, double targetAngle) {
        this.armSubsystem =armSubsystem;
        this.targetAngle = targetAngle;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        armSubsystem.setArm(targetAngle);
    }

}
