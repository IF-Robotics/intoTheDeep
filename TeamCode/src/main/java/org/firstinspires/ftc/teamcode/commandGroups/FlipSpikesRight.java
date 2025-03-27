package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armAutoPushY;
import static org.firstinspires.ftc.teamcode.other.Globals.armAutoReadyPushY;
import static org.firstinspires.ftc.teamcode.other.Globals.armAutoSpikeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armAutoSpikeXTWO;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideLeftSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideMidSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideMiddleSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideRightSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.rightSideRightSpikeFlip;
import static org.firstinspires.ftc.teamcode.other.Robot.intakeRightFrontHighChamberCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

public class FlipSpikesRight extends SequentialCommandGroup {
    public FlipSpikesRight(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
                new DriveToPointCommand(driveSubsystem, rightSideLeftSpikeFlip, 5, 5),
                new WaitCommand(100),
                new IntakeSub(armSubsystem, intakeSubsystem),
                new WaitCommand(320),
                new ParallelCommandGroup(
                        new FlipSample(armSubsystem, intakeSubsystem),
                        //second sample
                        new DriveToPointCommand(driveSubsystem, new Pose2d(51, -50, Rotation2d.fromDegrees(0)), 5, 5),
                        new DriveToPointCommand(driveSubsystem,  rightSideMidSpikeFlip, 5, 5)
                ),

                //wait
//                new WaitCommand(1000),
                // intake sample
                new WaitCommand(100),
                new IntakeSub(armSubsystem, intakeSubsystem),
                new WaitCommand(300),
                new FlipSample(armSubsystem, intakeSubsystem),
                //second sample
                new DriveToPointCommand(driveSubsystem,  new Pose2d(61, -50, Rotation2d.fromDegrees(0)), 5, 5),
                
                new DriveToPointCommand(driveSubsystem,  new Pose2d(54, -45, Rotation2d.fromDegrees(-37)), 5, 5),
                // Third sample
                // arm up
                new ArmCoordinatesCommand(armSubsystem, armAutoSpikeXTWO, armAutoReadyPushY),
                new DriveToPointCommand(driveSubsystem,  new Pose2d(56, -36, Rotation2d.fromDegrees(-37)), 2, 5),
                new InstantCommand(() -> armSubsystem.setArmY(armAutoPushY)),

                new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.DOWN)),
                new DriveToPointCommand(driveSubsystem,  new Pose2d(46, -48, Rotation2d.fromDegrees(-120)), 10, 10),
                new WaitCommand(100));

                // intake sample
    }
}
