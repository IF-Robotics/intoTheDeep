package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

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
    public void initialize() {
        manualArm = true;
        new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.DOWN)).schedule();
    }

    @Override
    public void execute() {
        armSubsystem.manualArm(armPower.getAsDouble(), slidePower.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.UP)).schedule();
        manualArm = false;
        manualSlides = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

