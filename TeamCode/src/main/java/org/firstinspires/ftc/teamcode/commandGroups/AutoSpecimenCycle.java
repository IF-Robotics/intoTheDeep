package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class AutoSpecimenCycle extends SequentialCommandGroup {
    public AutoSpecimenCycle(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
        addCommands(
                // drive to specimen on wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -53, Rotation2d.fromDegrees(180)), 1, 5),
                //wait
                new WaitCommand(300),
                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -56, Rotation2d.fromDegrees(180)), 1, 5),
                //wait
                new WaitCommand(1000),
                // Intake specimen from wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -55, Rotation2d.fromDegrees(180)), 1, 5),
                //wait
                new WaitCommand(1000),
               // set wrist to ready position for high chamber
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontRightHighChamber, 130),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontRightHighChamber, rollFrontRightHighChamber),
                new WaitCommand(200),
                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),
                new WaitCommand(200),
                // Drive to high chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(6.9, -45, Rotation2d.fromDegrees(180)),3, 5),
                new DriveToPointCommand(driveSubsystem, highChamberRight ,3, 5),
                //wait
                new WaitCommand(1000),
//                new DriveToPointCommand((driveSubsystem, new Pose2d(-53, -54, Rotation2d.fromDegrees(-45)), 1, 5, 10000),
                new WaitCommand(300),
                // Score specimen
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
                new WaitCommand(300),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
                // reset arm and slides
                new InstantCommand(() -> armSubsystem.setArm(10)),
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                // reset diffy wrist
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                new WaitCommand(300)

                  //new DriveToPointCommand(driveSubsystem, new Pose2d(35, -53, Rotation2d.fromDegrees(180)), 1, 5,1000),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -56, Rotation2d.fromDegrees(180)), 1, 5,1000),
//                new WaitCommand(500),
//                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -55, Rotation2d.fromDegrees(180)), 1, 5,1000),
//                new InstantCommand(() -> armSubsystem.setArm(50)),
//                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, 130),
//                new WaitCommand(200),
//                intakeRightFrontHighChamberCommand,
//                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),
//                new WaitCommand(500),
//                // Drive to high
//                new DriveToPointCommand(driveSubsystem, new Pose2d(6.9, -45, Rotation2d.fromDegrees(180)),3, 5,0),
//                new DriveToPointCommand(driveSubsystem, highChamberRight ,3, 5,1000),
////                new DriveToPointCommand((driveSubsystem, new Pose2d(-53, -54, Rotation2d.fromDegrees(-45)), 1, 5, 10000),
//                new WaitCommand(500),
//                // Score specimen
//                intakeRightScoreFrontHighChamberCommand,
//                new WaitCommand(500),
//                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
//                armBackCommand,
//                new InstantCommand(() -> armSubsystem.setArm(10)),
//                new InstantCommand(() -> armSubsystem.setSlide(8)),
//                new WaitCommand(500),

        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
