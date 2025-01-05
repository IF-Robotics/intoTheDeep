package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycle;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commandGroups.ScoreHighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.scoreHighBasket;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="5+1")
public class five_spec_oneSample_auto extends Robot {

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
                new InstantCommand(() -> armSubsystem.setArm(18)),
                //wait
                new WaitCommand(200),
                //extend slides
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY),
                //wait
                new WaitCommand(500),


                // Drive to high chamber
                // Score specimen
                new DriveToPointCommand(driveSubsystem, firstHighChamberRight,5, 10).withTimeout(1500),
                //open
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                new WaitCommand(100),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                new InstantCommand(() -> armSubsystem.setArm(45)),


                // Drive to middle
                new DriveToPointCommand(driveSubsystem, new Pose2d(12.45, -48, new Rotation2d(-45)), 10, 10),
                new ArmCoordinatesCommand(armSubsystem, 15, armAutoReadyPushY),


                //first sample
                new ParallelDeadlineGroup(new DriveToPointCommand(driveSubsystem, new Pose2d(29, -37, Rotation2d.fromDegrees(-37)), 5, 5),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),
                                intakeRightFrontHighChamberCommand
                        )
                ),
                // intake down
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, new Pose2d(34, -47, Rotation2d.fromDegrees(-120)), 5, 10),
                //arm up
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),




                //second sample
                new DriveToPointCommand(driveSubsystem,  rightSideMiddleSpike, 5, 5),
                //wait
//                new WaitCommand(1000),
                // intake sample
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                // wait?
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem,  new Pose2d(38.5, -45, Rotation2d.fromDegrees(-140)), 5, 5),
                // Third sample
                // arm up
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),
                new DriveToPointCommand(driveSubsystem,  rightSideRightSpike, 2, 5),
                // intake sample
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                // wait?
                new DriveToPointCommand(driveSubsystem,  new Pose2d(42, -45, Rotation2d.fromDegrees(-140)), 10, 10),

                //open claw
                //retract slide
                armWhenIntakeWallCommand,
                intakeWallCommand,
                //drive close to pickup point
                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -50, Rotation2d.fromDegrees(180)), 2, 5),

                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycle(armSubsystem, intakeSubsystem, driveSubsystem),





                //grab the yellow sample
                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),


                // drive to specimen on wall
//                new DriveToPointCommand(driveSubsystem, new Pose2d(33, -50, Rotation2d.fromDegrees(180)), 2, 5),
                //wait
//                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, wallPickUp, 1, 3),
                //wait
                new WaitCommand(100),


                // Intake specimen from wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -56, Rotation2d.fromDegrees(180)), 1, 5).withTimeout(200),
                //wait
                new WaitCommand(100),
                // set wrist to ready position for high chamber
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontRightHighChamber, rollFrontRightHighChamber),
                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),



                //score in the high basket
                //arm and intake to high basket
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(500),
                //drop sample & arm down
                new RetractFromBasket(armSubsystem, intakeSubsystem),




                //park
                new DriveToPointCommand(driveSubsystem, new Pose2d(40, -56, Rotation2d.fromDegrees(-90)), 1, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-90)), 10, 5)
        ));



    }


}
