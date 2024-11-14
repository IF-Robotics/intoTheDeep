package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.testHeading;
import static org.firstinspires.ftc.teamcode.other.Globals.testX;
import static org.firstinspires.ftc.teamcode.other.Globals.testY;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosRight;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;

@Autonomous(name="autoTest")
public class autoTest extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
                new DriveToPointCommand(driveSubsystem, new Pose2d(testX, testY, new Rotation2d(.1)) ,0, 0,1000000)
        ));

    }

}
