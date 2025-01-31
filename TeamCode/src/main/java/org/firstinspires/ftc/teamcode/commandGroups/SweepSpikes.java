package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armAutoPushY;
import static org.firstinspires.ftc.teamcode.other.Globals.armAutoReadyPushY;
import static org.firstinspires.ftc.teamcode.other.Globals.armAutoSpikeX;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideMiddleSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideRightSpike;
import static org.firstinspires.ftc.teamcode.other.Robot.intakeRightFrontHighChamberCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class SweepSpikes extends SequentialCommandGroup {
    public SweepSpikes(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
                // Drive to middle
                new InstantCommand(() -> armSubsystem.setArm(20)),
                //first sample
                new ParallelDeadlineGroup(
                        new DriveToPointCommand(driveSubsystem, rightSideLeftSpike, 5, 5),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY+1),
                                intakeRightFrontHighChamberCommand
                        )
                ),
                // intake down
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                new WaitCommand(300),
                new DriveToPointCommand(driveSubsystem, new Pose2d(34, -47, Rotation2d.fromDegrees(-120)), 5, 10),
                //arm up
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),




                //second sample
                new DriveToPointCommand(driveSubsystem,  rightSideMiddleSpike, 5, 5),
                //wait
//                new WaitCommand(1000),
                // intake sample
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),
                // wait?
                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem,  new Pose2d(38.5, -45, Rotation2d.fromDegrees(-130)), 5, 5),
                // Third sample
                // arm up
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeX, armAutoReadyPushY),
                new DriveToPointCommand(driveSubsystem,  rightSideRightSpike, 2, 5),
                // intake sample
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)));
    }
}
