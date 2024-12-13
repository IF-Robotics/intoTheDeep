package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;
import static org.firstinspires.ftc.teamcode.other.Robot.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class AutoSpecimenCycle extends SequentialCommandGroup {
    public AutoSpecimenCycle(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
        addCommands(

                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),
                new DriveToPointCommand(driveSubsystem, new Pose2d(33, -50, Rotation2d.fromDegrees(180)), 1, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -53.5, Rotation2d.fromDegrees(180)), 1, 5),


                // drive to specimen on wall
                new DriveToPointCommand(driveSubsystem, new Pose2d(33, -50, Rotation2d.fromDegrees(180)), 2, 5),
                //wait
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -56, Rotation2d.fromDegrees(180)), 1, 5),
                //wait
                new WaitCommand(200),


                // Intake specimen from wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -55, Rotation2d.fromDegrees(180)), 1, 5),
                //wait
                new WaitCommand(200),
               // set wrist to ready position for high chamber
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontRightHighChamber, rollFrontRightHighChamber),
                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),



                // Drive to high chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(9.6, -35, Rotation2d.fromDegrees(180)),10, 10),
                new DriveToPointCommand(driveSubsystem, new Pose2d(10, -32, Rotation2d.fromDegrees(180)),3, 5).withTimeout(1000),
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, highChamberRight ,3, 5),
                //wait
                new WaitCommand(200),
                // Score specimen


                //score the specimen
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber),
                //retract slides slightly
                new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY -2 ),
                //wait
                new WaitCommand(200),
                //open the claw
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber)

        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
