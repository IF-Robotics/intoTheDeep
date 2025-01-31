package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose2;


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

public class CycleLeftSpikeMarksFast extends SequentialCommandGroup {

    public CycleLeftSpikeMarksFast(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        addCommands(

                //1st spike
                //arm&intake to close intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),

                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, new Pose2d(-46, -38.5, Rotation2d.fromDegrees(0)),2, 5),
                new WaitCommand(500),

                //grab and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                //arm basket
                new HighBasketCommand(armSubsystem, intakeSubsystem),
                new WaitCommand(500),

                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                new WaitCommand(300),

                //score in high basket
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem),




                //2nd spike
                //arm&intake to close intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),

                //drive to 2nd spike
                new DriveToPointCommand(driveSubsystem, new Pose2d(-56, -38.5, Rotation2d.fromDegrees(0)),2, 5),
                //wait
                new WaitCommand(200),
                //grab and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                //arm basket
                new HighBasketCommand(armSubsystem, intakeSubsystem),
                new WaitCommand(500),

                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                new WaitCommand(500),
                //score in high basket
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem),


                //3rd spike
                //arm&intake to close intake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake),
                new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY),

                //drive to 3rd spike
                new DriveToPointCommand(driveSubsystem, new Pose2d(-59.3, -35.5, Rotation2d.fromDegrees(35)),2, 5).withTimeout(1000),
                //wait
                new WaitCommand(200),
                //grab and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                //arm basket
                new HighBasketCommand(armSubsystem, intakeSubsystem),
                new WaitCommand(500),

                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                new WaitCommand(500),
                //score in high basket
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem)
        );

        addRequirements(intakeSubsystem, armSubsystem);
    }
}