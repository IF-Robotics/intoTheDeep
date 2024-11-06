package org.firstinspires.ftc.teamcode.opModes;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

@Autonomous(name="autoRight")
public class autoRight extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new DriveToPointCommand(driveSubsystem, new Pose2d(0, 0, new Rotation2d(0)) ,0, 0,0),
                new WaitCommand(200)
//                new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY),
                //intake



        ));

    }

}
