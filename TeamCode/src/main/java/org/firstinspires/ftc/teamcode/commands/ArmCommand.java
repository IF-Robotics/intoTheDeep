package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private DoubleSupplier targetAngle;

    public ArmCommand(ArmSubsystem armSubsystem, DoubleSupplier targetAngle) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        armSubsystem.setArm(targetAngle.getAsDouble()*100);
    }

}
