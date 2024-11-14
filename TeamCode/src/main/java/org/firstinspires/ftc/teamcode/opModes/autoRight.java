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
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;

@Autonomous(name="autoRight")
public class autoRight extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosRight)),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchFrontHighChamber, rollFrontHighChamber),
                new DriveToPointCommand(driveSubsystem, new Pose2d(3.37, 24.4, Rotation2d.fromDegrees(-34.9)) ,1, 5,10000),
                new WaitCommand(200),
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, armFrontHighChamberY),
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, new Pose2d(20, 20, Rotation2d.fromDegrees(0)), 1, 5,10000)




        ));

    }

}
