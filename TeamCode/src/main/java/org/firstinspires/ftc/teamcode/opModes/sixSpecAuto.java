package org.firstinspires.ftc.teamcode.opModes;
import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleSlow;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.rightPreloadSpecScore;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointDoubleSupplierCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;

@Disabled
@Autonomous(name="6+0")

public class sixSpecAuto extends AutoBase {

    private double subX = 0;
    private double subY = 7.5;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

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
                new DriveToPointDoubleSupplierCommand(driveSubsystem, ()-> MathUtils.clamp(subX, -20, 4)+3, ()->firstHighChamberRight.getY(), firstHighChamberRight.getRotation(), 5, 10).withTimeout(1500),
                //open
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, autoPitchFrontHighChamber, rollFrontHighChamber),
                new WaitCommand(50),
                //arm to home pos
                new InstantCommand(() -> armSubsystem.setArm(60)),
                new WaitCommand(100),
                new InstantCommand(()->armSubsystem.setSlide(7.75)),


                //drive back
                new ParallelCommandGroup(
                    new DriveToPointDoubleSupplierCommand(driveSubsystem, ()->subX+3, ()->firstHighChamberRight.getY() - 5, firstHighChamberRight.getRotation(), 5, 5),
                    //extend slides into sub
                    new WaitForArmCommand(armSubsystem, -5, 5),
                    new WaitCommand(300).andThen(new InstantCommand(() -> intakeSubsystem.setDiffy(0,-50)))
                ).withTimeout(500),
                new InstantCommand(() -> intakeSubsystem.setDiffy(0,-50)),
                new ParallelCommandGroup(
                    new IntakeSub(armSubsystem, intakeSubsystem),
                    new DriveToPointDoubleSupplierCommand(driveSubsystem, ()->subX+3, ()->-46.5 + subY, firstHighChamberRight.getRotation(), 5, 5)
                ),
                new WaitCommand(800).interruptOn(()->armSubsystem.getCurrentX()>armReadySubIntakeX-0.75),
                //vision
                new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, true).withTimeout(2000),
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
        while(!isStarted() && !isStopRequested()){
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                subX -= 1;
                MathUtils.clamp(subX, -8, 6);
            }

            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                subX += 1;
                subX = MathUtils.clamp(subX, -8, 6);
            }

            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                subY += 1;
                subY = MathUtils.clamp(subY, 0, 15);
            }

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                subY -= 1;
                subY = MathUtils.clamp(subY, 0, 15);
            }

            telemetry.addData("subX (-8,6)", subX);
            telemetry.addData("subY(offesetFromBarrier)(0,15)", subY);
            telemetry.update();
        }
    }
}
