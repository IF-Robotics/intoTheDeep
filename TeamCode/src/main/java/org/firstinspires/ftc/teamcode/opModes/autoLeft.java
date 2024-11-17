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
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, armFrontHighChamberY),
                //drive to high chamber
                new DriveToPointCommand(driveSubsystem, highChamberLeft ,1, 5,1000),
                new WaitCommand(1000),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                new WaitCommand(1000),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideRightSpike,1.5, 5,500),
                new WaitCommand(1000),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(500),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(1000),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 1, 5, 1000),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(1000),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand//,


                //drive to second sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideMidSpike,1, 5,1000),
//                new DriveToPointCommand(driveSubsytem, )
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                new WaitCommand(1000),
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
                new WaitCommand(1000),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to third sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideRightSpike,1, 5,1000),
                intakeLastLeftAutoCommand, //find positions
                armWhenCloseIntakeCommand,
                new WaitCommand(1000),
                //grab and retract
                retractAfterIntake,
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 1, 5, 1000),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(1000),
                // park
                armLeftAutoParkCommand, // find position
                new DriveToPointCommand(driveSubsystem, leftAutoPark, 1, 5, 1000) //tune position





        ));

    }

}
