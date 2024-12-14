package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycle;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="5+0")
public class five_spec_auto extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();

        //schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber));
        manualArm = false;

        new InstantCommand(() -> armSubsystem.setArm(90)).schedule(true);
        claw.setPosition(clawClose);


        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));




        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosRight)),
                //wait
                new WaitCommand(6),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosRight)),

                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setArm(35)),
                //wait
                new WaitCommand(300),
                //extend slides
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY),
                //wait
                new WaitCommand(500),


                // Drive to high chamber
                // Score specimen
                new DriveToPointCommand(driveSubsystem, firstHighChamberRight,5, 10).withTimeout(2000),
                //wait
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(8)),


                // Drive to middle
                new DriveToPointCommand(driveSubsystem, new Pose2d(12.45, -48, new Rotation2d(-37)), 10, 20),


                //first sample
                intakeRightFrontHighChamberCommand,
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),
                new DriveToPointCommand(driveSubsystem, new Pose2d(30, -37, Rotation2d.fromDegrees(-37)), 2, 2),
                //wait
//                new WaitCommand(500),
                // intake down
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, new Pose2d(34, -47, Rotation2d.fromDegrees(-120)), 5, 5),
                //arm up
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),




                //second sample
                new DriveToPointCommand(driveSubsystem,  new Pose2d(38.5, -37, Rotation2d.fromDegrees(-37)), 1, 2),
                //wait
//                new WaitCommand(1000),
                // intake sample
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                new WaitCommand(200),
                // wait?
                new DriveToPointCommand(driveSubsystem,  new Pose2d(38.5, -45, Rotation2d.fromDegrees(-140)), 5, 5),
                // Third sample
                // arm up
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),
                new DriveToPointCommand(driveSubsystem,  new Pose2d(45, -37, Rotation2d.fromDegrees(-37)), 1, 2),
                new DriveToPointCommand(driveSubsystem,  new Pose2d(48.5, -37, Rotation2d.fromDegrees(-37)), 1, 2),
                //wait
//                new WaitCommand(1000),
                // intake sample
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                new WaitCommand(200),
                // wait?
                new DriveToPointCommand(driveSubsystem,  new Pose2d(42, -45, Rotation2d.fromDegrees(-140)), 5, 5),

                //open claw
                //retract slide
                armWhenIntakeWallCommand,
                intakeWallCommand,

                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-180)), 1, 5)




                //cycle
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -53, Rotation2d.fromDegrees(180)), 1, 5,1000),
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
//
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -53, Rotation2d.fromDegrees(180)), 1, 5,1000),
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
//                new WaitCommand(500)
        ));



    }


}
