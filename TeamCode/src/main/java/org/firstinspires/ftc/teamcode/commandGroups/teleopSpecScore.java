package org.firstinspires.ftc.teamcode.commandGroups;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
public class teleopSpecScore extends SequentialCommandGroup{
    public teleopSpecScore(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> driveSubsystem.setStartingPos(wallPickUp)),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem),
                            new AutoSpecimenCycle2(armSubsystem,intakeSubsystem,driveSubsystem)
                        ),
                        new AutoDriveCommand(driveSubsystem)
                )


        );
        addRequirements(driveSubsystem, armSubsystem, intakeSubsystem);
    }
}
