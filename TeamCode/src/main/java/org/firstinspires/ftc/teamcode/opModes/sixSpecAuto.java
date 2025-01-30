package org.firstinspires.ftc.teamcode.opModes;
import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleSlow;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.rightPreloadSpecScore;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;

@Disabled
@Autonomous(name="6+0")

public class sixSpecAuto extends AutoBase {

@Override
    public void initialize() {
        super.initialize();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosRight)),
                //wait
                new WaitCommand(6),

                //hold pos
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosRight)),

                //score preload
                //raise intake and arm
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, autoPitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setArm(22)),
                //wait
                new WaitCommand(200),
                //extend slides
                new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, autoArmFrontHighChamberY),
                //wait
                new WaitCommand(300),


                // Drive to high chamber
                // Score specimen
                new DriveToPointCommand(driveSubsystem, firstHighChamberRight,5, 10).withTimeout(1500),
                //open
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                new WaitCommand(50),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setArm(40)),
                new WaitCommand(100),
                new InstantCommand(()->armSubsystem.setSlide(7.75)),


                //intake from sub
                //drive translationally to spot indicated at by
                new ParallelCommandGroup(
                    new DriveToPointCommand(driveSubsystem, new Pose2d(0, firstHighChamberRight.getY()-3, firstHighChamberRight.getRotation()), 5, 5),
                    //extend slides into sub
                    new WaitForArmCommand(armSubsystem, 0, 5)
                ).withTimeout(500),
                new IntakeSub(armSubsystem, intakeSubsystem),
                new InstantCommand(() -> intakeSubsystem.setDiffy(0,0)),
                new WaitCommand(800).interruptOn(()->armSubsystem.getCurrentX()>armReadySubIntakeX-1.5),
                //vision
                new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, true).withTimeout(1500),
                //wait
                new WaitCommand(100),
                //pickup sample and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),



                //dropoff sample at observation zone
                //drive to observation zone
                new ParallelDeadlineGroup(
                    new DriveToPointCommand(driveSubsystem, new Pose2d(45, -50, new Rotation2d(Math.toRadians(15))), 10, 10),
                    new SequentialCommandGroup(
                        new WaitCommand(400),
                        new InstantCommand(() -> armSubsystem.setArm(95)),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket)
                    )
                ),
                //drop sample
                new InstantCommand(() -> intakeSubsystem.openClaw()),



                //sweep spikes
                new SweepSpikes(driveSubsystem, armSubsystem, intakeSubsystem),



                new WaitCommand(300),
                // wait?
                new DriveToPointCommand(driveSubsystem,  new Pose2d(42, -45, Rotation2d.fromDegrees(-140)), 10, 10),
                //open claw
                //retract slide
                armWhenIntakeWallCommand,
                intakeWallCommand,
                //drive close to pickup point
                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -50, Rotation2d.fromDegrees(180)), 2, 5),

                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleFast(armSubsystem, intakeSubsystem, driveSubsystem),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-180)), 1, 5)
        ));

        //gamepad input
        while(opModeInInit()){

        }
    }
}
