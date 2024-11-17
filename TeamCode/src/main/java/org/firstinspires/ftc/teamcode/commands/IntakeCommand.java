package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private double pitchAngle;
    private double rollAngle;

    private Claw claw;
    public enum Claw {
        OPEN,
        CLOSE,
        EXTRAOPEN
    }

    public IntakeCommand(IntakeSubsystem intakeSubsystem, Claw claw , double pitchAngle, double rollAngle) {
        this.intakeSubsystem = intakeSubsystem;
        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;
        this.claw = claw;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {

        switch (claw) {
            case OPEN:
                intakeSubsystem.openClaw();
                break;
            case CLOSE:
                intakeSubsystem.closeClaw();
            case EXTRAOPEN:
                intakeSubsystem.extraOpenClaw();
                break;
        }

        intakeSubsystem.setDiffy(pitchAngle, rollAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
