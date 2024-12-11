package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenBasket;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class scoreHighBasket extends SequentialCommandGroup {


    public scoreHighBasket(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //arm and intake to high basket
                                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket),
                                //drive to in front of high basket
                                new DriveToPointCommand(driveSubsystem, leftBasketPose.transformBy(new Transform2d(new Translation2d(0, 6), new Rotation2d(0))), 4, 10),
                                //wait for slides
                                new WaitForSlideCommand(armSubsystem, 38.5, 2),
                                //drive to high basket
                                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                                //wait
                                new WaitCommand(1500),
                                //drop sample & arm down
                                new RetractFromBasket(armSubsystem, intakeSubsystem)
                        ),
                        new holdDTPosCommand(driveSubsystem)
                )
                );


        addRequirements(driveSubsystem, armSubsystem, intakeSubsystem);
    }
}
