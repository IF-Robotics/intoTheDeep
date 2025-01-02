
package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.armFrontHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.autoArmFrontHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.autoPitchFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenBasket;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftAutoPark;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose2;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosLeft;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosLeft2;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.CycleLeftSpikeMarks;
import org.firstinspires.ftc.teamcode.commandGroups.CycleLeftSpikeMarks2;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;

@Autonomous(name="0+4")
public class zeroPlusFourAuto extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();

        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));

        manualArm = false;


        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft2)),
                //wait
                new WaitCommand(6),
                //stay at startpoint
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosLeft2)),

                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                new DriveToPointCommand(driveSubsystem, new Pose2d(-15, -50, Rotation2d.fromDegrees(0)), 2, 5),
                //raise intake and arm
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 42),
                new WaitCommand(300),
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, 40),
                new WaitCommand(300),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 390, rollWhenBasket),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, 390, rollWhenBasket),
                new WaitCommand(200),

                //cycle the spikemarks
                new CycleLeftSpikeMarks2(driveSubsystem, intakeSubsystem, armSubsystem),

                // park
                //move arm up
                armLeftAutoParkCommand, // find position
                //drive away from basket
                new DriveToPointCommand(driveSubsystem, new Pose2d(-50, -7, Rotation2d.fromDegrees(-90)), 5, 5),
                //drive to low bar
                new DriveToPointCommand(driveSubsystem, leftAutoPark, 3, 10).withTimeout(3000),//tune position*/
                new RunCommand(() -> armSubsystem.setPowerZero(), armSubsystem)
        ));



    }


}