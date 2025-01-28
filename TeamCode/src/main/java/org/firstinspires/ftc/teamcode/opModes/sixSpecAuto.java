package org.firstinspires.ftc.teamcode.opModes;
import static org.firstinspires.ftc.teamcode.other.Globals.*;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleSlow;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;

@Autonomous(name="6+0")

public class sixSpecAuto extends AutoBase {

@Override
    public void initialize() {
        super.initialize();

        schedule(new SequentialCommandGroup(
                new StartSpecAuto(driveSubsystem, armSubsystem, intakeSubsystem),
                new InstantCommand(()->armSubsystem.setSlide(7.75)),


                //intake from sub
                //drive translationally to spot indicated at by
                new DriveToPointCommand(driveSubsystem, new Pose2d(0, firstHighChamberRight.getY()-3, firstHighChamberRight.getRotation()), 5, 5),
                new WaitCommand(1000),
                //extend slides into sub
                new WaitForArmCommand(armSubsystem, 0, 5),
                new IntakeSub(armSubsystem, intakeSubsystem),
                new InstantCommand(() -> intakeSubsystem.setDiffy(0,0)),
                new WaitCommand(1000),
                new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, true).withTimeout(30000),
                //vison code
                //TODO: add vision code
                //pickup sample and retract
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                new WaitCommand(1000),
                new InstantCommand(() -> armSubsystem.setArm(95)),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket),



                //dropoff sample at observation zone
                //drive to observation zone
                new DriveToPointCommand(driveSubsystem, new Pose2d(45, -50, new Rotation2d(0)), 10, 10),
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

                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleSlow(armSubsystem, intakeSubsystem, driveSubsystem),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-180)), 1, 5)
        ));

        //gamepad input
        while(opModeInInit()){

        }
    }
}
