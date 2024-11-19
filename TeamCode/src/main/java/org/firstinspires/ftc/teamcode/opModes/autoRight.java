package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.other.Robot;


@Autonomous(name="autoRight")
public class autoRight extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();


        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosRight)),
                new WaitCommand(1000),
                //raise intake and arm
                intakeRightFrontHighChamber,
                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),
                // Drive to high chamber
                new DriveToPointCommand(driveSubsystem, highChamberRight ,1, 5,10000),
//                new DriveToPointCommand((driveSubsystem, new Pose2d(-53, -54, Rotation2d.fromDegrees(-45)), 1, 5, 10000),
                // Score specimen
                intakeRightScoreFrontHighChamber,
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
                new WaitCommand(100),
                // Drive to middle

//                new DriveToPointCommand(driveSubsystem, new Pose2d(20, 20, Rotation2d.fromDegrees(0)), 1, 5,10000),
                // intake right left specimen
                intakeCloseCommand,
                armWhenCloseIntakeCommand





        ));

    }

}
