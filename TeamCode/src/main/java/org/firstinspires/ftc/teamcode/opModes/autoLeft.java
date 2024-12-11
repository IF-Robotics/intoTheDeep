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

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="autoLeft")
public class autoLeft extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();

        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));


        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft)),
                //wait
                new WaitCommand(6),
                //stay at startpoint
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosLeft)),

                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setArm(20)),
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY),
                //wait
                new WaitCommand(800),
                //drive to high chamber
                new DriveToPointCommand(driveSubsystem, highChamberLeft,5, 10).withTimeout(2000),
                //wait
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                //wait
                new WaitCommand(400),
                // back up from front chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(-17.36, -45, Rotation2d.fromDegrees(0)),  5, 10),
                //wait
                new WaitCommand(300),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideRightSpike,2, 5),
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(400),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(400),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(500),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(500),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to second sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideMidSpike,2, 5),
                //wait
                new WaitCommand(200),
//                new DriveToPointCommand(driveSubsytem, )
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                new WaitCommand(300),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(400),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(500),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(300),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(300),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to third sample on the spikemark
                intakeLastLeftAutoCommand,
                new DriveToPointCommand(driveSubsystem, leftSideLeftSpike,2, 5),
                armWhenCloseIntakeCommand,
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(500),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(500),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(500),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(500),
                // park
                armLeftAutoParkCommand, // find position

                new DriveToPointCommand(driveSubsystem, new Pose2d(-50, -7, Rotation2d.fromDegrees(-90)), 5, 5),

                new DriveToPointCommand(driveSubsystem, leftAutoPark, 3, 10).withTimeout(3000),//tune position*/
                new RunCommand(() -> armSubsystem.setPowerZero(), armSubsystem)
        ));



    }


}
