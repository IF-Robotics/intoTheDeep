
package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.CycleLeftSpikeMarksFast;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasketAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SampleSubAuto;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;

@Autonomous(name="0+6 \uD83D\uDFE1˚˖\uD80C\uDF62ִִ໋\uD83C\uDF0A\uD83E\uDD88˚˖\uD80C\uDF62ִ✧˚.")
public class sampleSubCycle extends AutoBase {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();
 
        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));

        manualArm = false;

        schedule(new InstantCommand(()->intakeSubsystem.setDiffy(pitchWhenBasket, rollWhenBasket + 30)));


        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosLeft2)),
                //wait
                new WaitCommand(6),
                //stay at startpoint
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosLeft2)),


                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket),
                new WaitForArmCommand(armSubsystem, 90, 50),
                //extend slides
                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),


                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose2, 2, 5),

                new WaitCommand(100),

                //score in high basket
                new RetractFromBasketAuto(armSubsystem, intakeSubsystem),

                //cycle the spikemarks
                new CycleLeftSpikeMarksFast(driveSubsystem, intakeSubsystem, armSubsystem),

                //cyling from the sub
                new SampleSubAuto(driveSubsystem, intakeSubsystem, armSubsystem, visionSubsystem, colorSubsystem, new Pose2d(-26, -7, Rotation2d.fromDegrees(-90))),
                new SampleSubAuto(driveSubsystem, intakeSubsystem, armSubsystem, visionSubsystem, colorSubsystem, new Pose2d(-26, -3, Rotation2d.fromDegrees(-90))),
                new SampleSubAuto(driveSubsystem, intakeSubsystem, armSubsystem, visionSubsystem, colorSubsystem, new Pose2d(-26, -3, Rotation2d.fromDegrees(-90))),





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