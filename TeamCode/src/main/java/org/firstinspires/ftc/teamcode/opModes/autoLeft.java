package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.basketPose;
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
                new DriveToPointCommand(driveSubsystem, new Pose2d(-17.36, -40.2, Rotation2d.fromDegrees(-34)) ,1.5, 5,1000),
                new WaitCommand(200),
                new ArmCoordinatesCommand(armSubsystem, armHomeX, armHomeY),
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, new Pose2d(-46.9, -45.36, Rotation2d.fromDegrees(0)) ,1.5, 5,1000),
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                // arm home command?
                armHighBasketCommand,
                // drive to basket
                intakeWhenHighBasketCommand,
                intakeWhenArmBackCommand,
                armHomeCommand,
                // drive to other specimen
                intakeCloseCommand,
                armWhenCloseIntakeCommand,







//                new DriveToPointCommand(driveSubsystem, new Pose2d(20, 20, Rotation2d.fromDegrees(0)), 1, 5,1000
        ));

    }

}
