package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;

@Autonomous(name="autoRight")
public class autoRight extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(new Pose2d(startRightX, startY, Rotation2d.fromDegrees(0)))),
                new DriveToPointCommand(driveSubsystem, new Pose2d(0, 20, Rotation2d.fromDegrees(0)) ,1, 5,10000),
                new WaitCommand(200),
                new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY)
                //intake



        ));

    }

}
