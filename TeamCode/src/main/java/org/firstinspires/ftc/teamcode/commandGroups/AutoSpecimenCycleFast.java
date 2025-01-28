package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class AutoSpecimenCycleFast extends SequentialCommandGroup {
    public AutoSpecimenCycleFast(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
        addCommands(

                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),


                // drive to specimen on wall
//                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -46, Rotation2d.fromDegrees(180)), 20, 5),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -50, Rotation2d.fromDegrees(180)), 20, 5),

                //wait
//                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, wallPickUp, 1, 3).withTimeout(1000),
                //wait
                new WaitCommand(100),


                // Intake specimen from wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -56, Rotation2d.fromDegrees(180)), 1, 5).withTimeout(200),
                //wait
                new WaitCommand(100),
               // set wrist to ready position for high chamber
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontRightHighChamber, rollFrontRightHighChamber),
                new ArmCoordinatesCommand(armSubsystem, armRightHighChamberX, armRightHighChamberY),



                // Drive to high chamber
                new DriveToPointCommand(driveSubsystem, new Pose2d(9, -32+0.5, Rotation2d.fromDegrees(180)),3, 5).withTimeout(1500),
                new DriveToPointCommand(driveSubsystem, highChamberRight ,1, 5).withTimeout(500),
                //wait
//                new WaitCommand(200),
                // Score specimen


                //score the specimen
                new ScoreHighChamberCommand(armSubsystem, intakeSubsystem)

        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
