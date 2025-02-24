package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.clawClose;
import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollEvwall;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.startingPosRight;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleEV;
import org.firstinspires.ftc.teamcode.commandGroups.AutoSpecimenCycleSlow;
import org.firstinspires.ftc.teamcode.commandGroups.StartSpecAuto;
import org.firstinspires.ftc.teamcode.commandGroups.SweepSpikes;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

@Autonomous(name="EV")
public class Seven_specEv extends Robot {

    @Override
    public void initialize(){
        super.initialize();
        slideLeft.resetEncoder();

        //schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber));
        manualArm = false;

        new InstantCommand(() -> armSubsystem.setArm(90)).schedule(true);
        claw.setPosition(clawClose);



        //turn on auto drive
        driveSubsystem.setDefaultCommand(new holdDTPosCommand(driveSubsystem));

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(startingPosRight)),
                //wait
                new WaitCommand(6),

                //hold pos
                new ParallelCommandGroup(
                new InstantCommand(() -> driveSubsystem.driveToPoint(startingPosRight)),
                new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.DOWN))
                ),
                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -34, Rotation2d.fromDegrees(0)), 2, 5),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollEvwall),//remove later maybe
                new WaitCommand(500),
                new AutoSpecimenCycleEV(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleEV(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleEV(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleEV(armSubsystem, intakeSubsystem, driveSubsystem),
                new AutoSpecimenCycleEV(armSubsystem, intakeSubsystem, driveSubsystem),
                new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY),
                new DriveToPointCommand(driveSubsystem, new Pose2d(50, -56, Rotation2d.fromDegrees(-180)), 1, 5)
        ));



    }


}
