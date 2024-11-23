package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycle;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="autoRight2")


public class autoRightTwo extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();

        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));


        schedule(new SequentialCommandGroup(

                //wait
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosRight)),
                //wait
                new WaitCommand(6),

                //raise intake and arm
                new InstantCommand(() -> armSubsystem.setArm(90)),
//                new WaitCommand(200),
                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),
//                new WaitCommand(300),
                intakeRightFrontHighChamberCommand,
                // Drive to high chamber
                new DriveToPointCommand(driveSubsystem, highChamberRight ,1, 5).withTimeout(2000),
                //wait
                new WaitCommand(1000),
//                new DriveToPointCommand(driveSubsystem, highChamberRight ,1, 5),

//                new DriveToPointCommand((driveSubsystem, new Pose2d(-53, -54, Rotation2d.fromDegrees(-45)), 1, 5, 10000),
//                new WaitCommand(200),
                // Score specimen
                intakeRightScoreFrontHighChamberCommand,
                new WaitCommand(500),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
                armBackCommand,
                new WaitCommand(500),

                // Drive to middle
                new DriveToPointCommand(driveSubsystem, new Pose2d(12.45, -48, new Rotation2d(-37)), 5, 10),
                new InstantCommand(() -> armSubsystem.setArm(0)),
                // sample intake positions
                intakeAutoRightCommand,
                new DriveToPointCommand(driveSubsystem, new Pose2d(33.3, -41.5, Rotation2d.fromDegrees(-37)), 1, 5),
                //wait
                new WaitCommand(1000),
                // intake sample
                new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armReadySubIntakeY),
                new WaitCommand(600),
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY - 3/4)),
                new WaitCommand(800),
                intakeAutoRightGrabCommand,
                //place in observation zone
                new WaitCommand(200),
                new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armReadySubIntakeY),
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, new Pose2d(33.8, -41.5, Rotation2d.fromDegrees(-140)), 1, 5),
                //wait
                new WaitCommand(1000),
                //open claw
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, 315, rollWhenIntake),
                new WaitCommand(200),
                //retract slide

                intakeAutoRightCommand,
                new DriveToPointCommand(driveSubsystem, new Pose2d(43.3, -41.5, Rotation2d.fromDegrees(-37)), 1, 5),
                //wait
                new WaitCommand(1000),
                // intake sample
                new WaitCommand(300),
                new InstantCommand(() -> armSubsystem.setArmY(armSubIntakeY -3/4)),
                new WaitCommand(300),
                intakeAutoRightGrabCommand,
                // wait?
                new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armReadySubIntakeY),
                new DriveToPointCommand(driveSubsystem, new Pose2d(43.8, -41.5, Rotation2d.fromDegrees(-140)), 1, 5),
                //wait
                new WaitCommand(1000),
                //open claw
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new WaitCommand(200),
                //retract slide
                new InstantCommand(() -> armSubsystem.setArm(10.5)),
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                intakeWallCommand,

                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new RunCommand(() -> armSubsystem.setPowerZero(), armSubsystem)

        ));



    }


}
