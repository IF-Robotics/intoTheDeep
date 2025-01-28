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

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleFast;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commandGroups.rightPreloadSpecScore;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.AutoBase;

@Autonomous(name="5+1")
public class five_spec_oneSample_auto extends AutoBase {

    @Override
    public void initialize(){
        super.initialize();

        new InstantCommand(() -> armSubsystem.setArm(90)).schedule(true);
        claw.setPosition(clawClose);

        SequentialCommandGroup RetractFromBasketCustom = new SequentialCommandGroup(
                //outtake
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollWhenBasket),
                //wait
                new WaitCommand(50),
                //move intake out of the way
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchIntakeWall, rollWhenIntake),
                //wait
                new WaitCommand(75),
                //retract slides
                new InstantCommand(() -> armSubsystem.setSlide(8)),
                new WaitCommand(150),
                //move arm down
                new ArmCoordinatesCommand(armSubsystem, armHomeX, armHomeY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, rollWhenArmHome)
        );



        schedule(new SequentialCommandGroup(
                new StartSpecAuto(driveSubsystem, armSubsystem, intakeSubsystem),
                new DriveToPointCommand(driveSubsystem, new Pose2d(12.45, -48, new Rotation2d(-45)), 10, 10),
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





                //wall intake
                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall),

                //drive to wall intake
                new DriveToPointCommand(driveSubsystem, wallPickUp, 1, 3),
                //wait
                new WaitCommand(100),


                // Intake specimen from wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollIntakeWall),
                //wait
                new WaitCommand(100),
                // set wrist to ready position for high chamber
                new InstantCommand(() -> armSubsystem.setArm(90)),

                //drive to high basket while extending slides to high basket
                new ParallelDeadlineGroup(
                        new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                //arm and intake to high basket
                                new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket)
                        )
                ),
                //wait
                new WaitCommand(500),
                //drop sample & arm down
                RetractFromBasketCustom,




                //park
                new DriveToPointCommand(driveSubsystem, new Pose2d(30, -56, Rotation2d.fromDegrees(-90)), 1, 5),
                new DriveToPointCommand(driveSubsystem, new Pose2d(40, -56, Rotation2d.fromDegrees(-90)), 10, 5)
        ));



    }

}
