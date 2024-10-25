package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmManualCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private DoubleSupplier armPower, slidePower;

    public ArmManualCommand(ArmSubsystem armSubsystem, DoubleSupplier armPower, DoubleSupplier slidePower) {
        this.armSubsystem = armSubsystem;
        this.armPower = armPower;
        this.slidePower = slidePower;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.manualArm(armPower.getAsDouble(), slidePower.getAsDouble());
    }

}

