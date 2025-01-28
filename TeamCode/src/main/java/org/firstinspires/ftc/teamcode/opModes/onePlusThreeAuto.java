package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.CycleLeftSpikeMarks;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="1+3")
public class onePlusThreeAuto extends Robot {

    @Override
    public void initialize(){
        super.initialize();


        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft2)),
                //wait
                new WaitCommand(6),
                //stay at startpoint
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosLeft)),

                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setArm(25)),
                //wait
                new WaitCommand(300),
                //extend slides
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY),
                //wait
                new WaitCommand(400),
                //drive to high chamber
                new DriveToPointCommand(driveSubsystem, highChamberLeft,5, 10).withTimeout(2000),
                //wait
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                new WaitCommand(100),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                new InstantCommand(() -> armSubsystem.setArm(45)),
                //wait
                new WaitCommand(400),
                // back up from front chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(-17.36, -45, Rotation2d.fromDegrees(0)),  5, 10),
                //wait
                new WaitCommand(300),

                //cycle the spikemarks
                new CycleLeftSpikeMarks(driveSubsystem, intakeSubsystem, armSubsystem),

                // park
                //move arm up
                armLeftAutoParkCommand, // find position
                //drive away from basket
                new DriveToPointCommand(driveSubsystem, new Pose2d(-50, -7, Rotation2d.fromDegrees(-90)), 5, 5),
                //drive to low bar
                new DriveToPointCommand(driveSubsystem, new Pose2d(-30, -7.09, Rotation2d.fromDegrees(-90)), 10, 10).withTimeout(1000),
                new DriveToPointCommand(driveSubsystem, leftAutoPark, 3, 10).withTimeout(500),//tune position*/
                new RunCommand(() -> armSubsystem.setPowerZero(), armSubsystem)
        ));



    }


}
