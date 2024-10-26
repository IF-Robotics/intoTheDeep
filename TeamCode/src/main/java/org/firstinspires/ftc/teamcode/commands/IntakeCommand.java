package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private double pitchAngle;
    private double rollAngle;
    private double power;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double power, double pitchAngle, double rollAngle) {
        this.intakeSubsystem = intakeSubsystem;
        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;
        this.power = power;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntake(power);
        intakeSubsystem.setDiffy(pitchAngle, rollAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
