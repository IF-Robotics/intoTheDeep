package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

public class ArmManualCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private DoubleSupplier armPower, slidePower;
    GamepadEx driverOp;

    public ArmManualCommand(ArmSubsystem armSubsystem, GamepadEx driverOp, DoubleSupplier armPower, DoubleSupplier slidePower) {
        this.armSubsystem = armSubsystem;
        this.driverOp = driverOp;
        this.armPower = armPower;
        this.slidePower = slidePower;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        manualArm = true;
        manualSlides = true;
    }

    @Override
    public void execute() {
        armSubsystem.manualArm(armPower.getAsDouble(), slidePower.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        if(driverOp.getButton(GamepadKeys.Button.BACK)) {
            manualArm = false;
            manualSlides = false;
            return true;
        } else {
            return false;
        }

    }

}

