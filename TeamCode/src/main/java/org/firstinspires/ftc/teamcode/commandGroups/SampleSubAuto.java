package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose2;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideLeftSpike;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

public class SampleSubAuto extends SequentialCommandGroup {
    public SampleSubAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem, Pose2d intakePose) {

        addCommands(
                new DriveToPointCommand(driveSubsystem, new Pose2d(-45, 3, Rotation2d.fromDegrees(-90)),2, 10).withTimeout(1000),
                new ParallelCommandGroup(
                    new DriveToPointCommand(driveSubsystem, intakePose,2, 5).withTimeout(500),
                    new WaitCommand(800)
                        .interruptOn(()->driveSubsystem.getAutoDriveError()<5)
                        .andThen(new InstantCommand(() -> intakeSubsystem.setDiffy(0,-50)))
                        .andThen(new IntakeSub(armSubsystem, intakeSubsystem))
                ),
                new WaitCommand(800).interruptOn(()->armSubsystem.getCurrentX()>armReadySubIntakeX-0.4),
                new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, true, ()->false,()->0, ()->0, ()->0, true).withTimeout(20000),
                new WaitCommand(100),
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                new ParallelCommandGroup(
                    new DriveToPointCommand(driveSubsystem, new Pose2d(-50, -7, Rotation2d.fromDegrees(-90)),2, 10).withTimeout(300)
                        .andThen(new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5)),
                    new HighBasketCommand(armSubsystem, intakeSubsystem)
                ),
                new WaitCommand(100),
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem)

        );
    }
}
