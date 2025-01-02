package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose2;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideLeftSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideMidSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideRightSpike;
import static org.firstinspires.ftc.teamcode.other.Robot.armHighBasketCommand;
import static org.firstinspires.ftc.teamcode.other.Robot.armWhenCloseIntakeCommand;
import static org.firstinspires.ftc.teamcode.other.Robot.intakeCloseCommand;
import static org.firstinspires.ftc.teamcode.other.Robot.intakeLastLeftAutoCommand;
import static org.firstinspires.ftc.teamcode.other.Robot.intakeWhenHighBasketCommand;
import static org.firstinspires.ftc.teamcode.other.Robot.retractAfterIntake;
import static org.firstinspires.ftc.teamcode.other.Robot.retractFromBasket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class CycleLeftSpikeMarks2 extends SequentialCommandGroup {

    public CycleLeftSpikeMarks2(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, new Pose2d(-46, -38.5, Rotation2d.fromDegrees(0)),2, 5),
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(400),
                //arm & intake to high basket
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 40),
                new WaitCommand(300),

                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                new WaitCommand(300),

                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 350, rollWhenBasket),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, 350, rollWhenBasket),
                new WaitCommand(200),
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 43),

                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(500),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to second sample on the spikemark
                new DriveToPointCommand(driveSubsystem, new Pose2d(-56, -38.5, Rotation2d.fromDegrees(0)),2, 5),
                //wait
                new WaitCommand(200),
//                new DriveToPointCommand(driveSubsytem, )
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                new WaitCommand(300),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(500),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 40),
                new WaitCommand(300),
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                new WaitCommand(300),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 390, rollWhenBasket),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, 390, rollWhenBasket),
                new WaitCommand(200),
                //drop sample & arm down
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 43),

                retractFromBasket,
                new WaitCommand(300),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to third sample on the spikemark
                intakeLastLeftAutoCommand,
                new DriveToPointCommand(driveSubsystem, new Pose2d(-59.3, -35.5, Rotation2d.fromDegrees(35)),2, 5),
                armWhenCloseIntakeCommand,
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(500),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 40),
                new WaitCommand(300),
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                new WaitCommand(300),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 390, rollWhenBasket),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, 390, rollWhenBasket),
                new WaitCommand(200),
                //drop sample & arm down
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 43),
                retractFromBasket,
                new WaitCommand(500)
        );

        addRequirements(intakeSubsystem, armSubsystem);
    }
}
