package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="autoLeft")
public class autoLeft extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();




        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft)),
                //wait
                new WaitCommand(6),

                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setArm(20)),
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY),
                //drive to high chamber
                new DriveToPointCommand(driveSubsystem, highChamberLeft,4, 10),
                //wait
                new WaitCommand(500),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                new WaitCommand(200),
                // back up from front chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(-17.36, -43, Rotation2d.fromDegrees(0)),  5, 10),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideRightSpike,2, 5),
                new WaitCommand(1000),
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
                new WaitCommand(1000),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(200),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to second sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideMidSpike,2, 5),
                //wait
                new WaitCommand(1000),
//                new DriveToPointCommand(driveSubsytem, )
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                new WaitCommand(400),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(400),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(600),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(1000),
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
                new WaitCommand(600),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(600),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(500),
                //drop sample & arm down
                retractFromBasket,
                // park
                new DriveToPointCommand(driveSubsystem, new Pose2d(-39.59, -7.52, Rotation2d.fromDegrees(-86.27)), 5, 5),
                armLeftAutoParkCommand, // find position
                new DriveToPointCommand(driveSubsystem, leftAutoPark, 3, 10) //tune position*/
        ));

    }

}
