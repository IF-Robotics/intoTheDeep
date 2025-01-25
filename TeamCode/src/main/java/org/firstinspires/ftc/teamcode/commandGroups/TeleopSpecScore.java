package org.firstinspires.ftc.teamcode.commandGroups;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class TeleopSpecScore extends SequentialCommandGroup{
    public TeleopSpecScore(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> driveSubsystem.setStartingPos(new Pose2d(wallPickUp.getX(), wallPickUp.getY(), driveSubsystem.getPos().getRotation()))),
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
