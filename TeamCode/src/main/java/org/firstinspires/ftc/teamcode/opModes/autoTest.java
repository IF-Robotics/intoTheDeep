package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.testX;
import static org.firstinspires.ftc.teamcode.other.Globals.testY;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="autoTest")
public class autoTest extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
                new DriveToPointCommand(driveSubsystem, new Pose2d(testX, testY, new Rotation2d(0)) ,0, 0,1000000)
        ));

    }

}
