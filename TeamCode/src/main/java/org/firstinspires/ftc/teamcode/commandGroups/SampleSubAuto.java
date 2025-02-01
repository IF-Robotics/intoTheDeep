package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose2;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideLeftSpike;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

public class SampleSubAuto extends SequentialCommandGroup {
    public SampleSubAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem, ColorSubsystem colorSubsystem, Pose2d intakePose) {

        addCommands(
                new DriveToPointCommand(driveSubsystem, new Pose2d(-52, 3, Rotation2d.fromDegrees(-90)),2, 7).withTimeout(1000),
                new ParallelCommandGroup(
                    new DriveToPointCommand(driveSubsystem, intakePose,2, 5).withTimeout(500),
                    new WaitCommand(800)
                        .interruptOn(()->driveSubsystem.getAutoDriveError()<5)
                        .andThen(new InstantCommand(() -> intakeSubsystem.setDiffy(0,-50)))
                        .andThen(new IntakeSub(armSubsystem, intakeSubsystem))
                ),
                new WaitCommand(800).interruptOn(()->armSubsystem.getCurrentX()>armReadySubIntakeX-0.4),
                new ParallelDeadlineGroup(
                    new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, true, ()->false,()->0, ()->0, ()->0, true),
                    new WaitCommand(400).andThen(new WaitCommand(100000000).interruptOn(()->!VisionToSampleInterpolate.hasFoundBlock)).andThen(
                        new DriveToPointCommand(driveSubsystem, new Pose2d(intakePose.getX(), intakePose.getY(), intakePose.getRotation().plus(new Rotation2d(Math.toRadians(30)))),2, 5).withTimeout(500)
                    )
                ).withTimeout(3500),
                new WaitCommand(100),
                new RetractAfterIntake(armSubsystem, intakeSubsystem, colorSubsystem),
                new ParallelCommandGroup(
                    new DriveToPointCommand(driveSubsystem, new Pose2d(-40, -7, Rotation2d.fromDegrees(-90)),2, 10).withTimeout(300)
                        .andThen(new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5)),
                    new HighBasketCommand(armSubsystem, intakeSubsystem)
                ),
                new WaitCommand(200),
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem)

        );
    }
}
