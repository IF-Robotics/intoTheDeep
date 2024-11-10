package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.basketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosRight;

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

@Autonomous(name="autoLeft")
public class autoLeft extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft)),
                //extend arm
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, armFrontHighChamberY),
                //drive to chamber
                new DriveToPointCommand(driveSubsystem, highChamberLeft, 1, 5, 1000),
                new WaitCommand(1000),
                //drive to baskets
                new DriveToPointCommand(driveSubsystem, basketPose, 1, 5, 1000),
                new WaitCommand(1000)

        ));

    }

}
