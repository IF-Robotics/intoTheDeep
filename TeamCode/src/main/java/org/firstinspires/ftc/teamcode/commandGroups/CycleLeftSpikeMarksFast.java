package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class CycleLeftSpikeMarksFast extends SequentialCommandGroup {


    public CycleLeftSpikeMarksFast(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        Command customHighBasketCommand = new SequentialCommandGroup(
                //move arm back
                new WaitForArmCommand(armSubsystem, 100, 70),

                //move to high basket
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket)
        );

        addCommands(

                //1st spike
                //arm&intake to close intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),

                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, new Pose2d(leftSideRightSpike.getX(), leftSideRightSpike.getY()-6, Rotation2d.fromDegrees(0)),2, 5),
                new DriveToPointCommand(driveSubsystem, leftSideRightSpike,2, 5),
                new WaitCommand(100),

                //grab and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                //arm basket
                customHighBasketCommand,

                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),

                //score in high basket
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem),




                //2nd spike
                //arm&intake to close intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),

                //drive to 2nd spike
                new DriveToPointCommand(driveSubsystem, new Pose2d(leftSideMidSpike.getX(), leftSideMidSpike.getY()-6, Rotation2d.fromDegrees(0)),3, 5),
                new DriveToPointCommand(driveSubsystem, leftSideMidSpike,2, 5),
                //wait
                new WaitCommand(200),
                //grab and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                //arm basket
                customHighBasketCommand,

                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                //score in high basket
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem),


                //3rd spike
                //arm&intake to close intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),

                //drive to 3rd spike
                new DriveToPointCommand(driveSubsystem, leftSideLeftSpike,2, 5).withTimeout(1000),
                //wait
                new WaitCommand(200),
                //grab and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                //arm basket
                customHighBasketCommand,

                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                //score in high basket
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem)
        );

        addRequirements(intakeSubsystem, armSubsystem);
    }
}