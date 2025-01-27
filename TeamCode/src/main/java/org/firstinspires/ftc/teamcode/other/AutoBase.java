package org.firstinspires.ftc.teamcode.other;

import static org.firstinspires.ftc.teamcode.other.Globals.clawClose;
import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;

public abstract class AutoBase extends Robot{

    public void initialize() {
        super.initialize();

        //reset encoders
        slideLeft.resetEncoder();
        slideRight.resetEncoder();

        //schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber));
        manualArm = false;

        new InstantCommand(() -> armSubsystem.setArm(90)).schedule(true);
        claw.setPosition(clawClose);


        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));
    }


}
