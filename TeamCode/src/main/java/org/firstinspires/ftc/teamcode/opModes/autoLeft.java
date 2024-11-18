package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;

@Autonomous(name="autoLeft")
public class autoLeft extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft)),
                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setArm(20)),
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, armFrontHighChamberY),
                //drive to high chamber
                new DriveToPointCommand(driveSubsystem, highChamberLeft ,3, 5,1000),
                new WaitCommand(100),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchFrontHighChamber, rollFrontHighChamber),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                new WaitCommand(300),
                // back up from front chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(-17.36, -43, Rotation2d.fromDegrees(0)),  5, 10,0),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideRightSpike,1.5, 5,500),
                new WaitCommand(800),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(500),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(500),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 1, 5, 1000),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(300),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to second sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideMidSpike,1, 5,1000),
//                new DriveToPointCommand(driveSubsytem, )
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(500),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(800),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 1, 5, 1000),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(300),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to third sample on the spikemark
                intakeLastLeftAutoCommand,
                new DriveToPointCommand(driveSubsystem, leftSideLeftSpike,1, 5,1000),
                armWhenCloseIntakeCommand,
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(800),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(800),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 1, 5, 1000),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(500)
                // park
//                armLeftAutoParkCommand, // find position
//                new DriveToPointCommand(driveSubsystem, leftAutoPark, 1, 5, 1000) //tune position





        ));

    }

}
