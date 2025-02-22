package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.Globals.armEvHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armEvHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.armEvIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armEvIntakewallY;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeWallY;
import static org.firstinspires.ftc.teamcode.other.Globals.armRightHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armRightHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchEvAutoHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchFrontRightHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchIntakeWall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollEvAutoHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollEvwall;
import static org.firstinspires.ftc.teamcode.other.Globals.rollFrontRightHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollIntakeWall;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberEvRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.highChamberRight;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallEvPickUp;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.wallPickUp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class AutoSpecimenCycleEV extends SequentialCommandGroup {
    public AutoSpecimenCycleEV(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
        addCommands(
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollEvwall),

//                new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.DOWN)),
                new ArmCoordinatesCommand(armSubsystem, armEvIntakeWallX, armEvIntakewallY),
//                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollEvwall),




                // drive to specimen on wall
//                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -46, Rotation2d.fromDegrees(180)), 20, 5),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(37, -50, Rotation2d.fromDegrees(180)), 20, 5),

                //wait
//                new WaitCommand(200),
                new DriveToPointCommand(driveSubsystem, new Pose2d(30, -56, Rotation2d.fromDegrees(40)), 1, 3).withTimeout(100),
                new DriveToPointCommand(driveSubsystem, new Pose2d(32, -54, Rotation2d.fromDegrees(0)), 1, 3).withTimeout(100),
                new ArmCoordinatesCommand(armSubsystem, armEvIntakeWallX, armEvIntakewallY),
                new DriveToPointCommand(driveSubsystem, wallEvPickUp, 1, 3).withTimeout(1300),
                //wait
                new WaitCommand(0),


                // Intake specimen from wall
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollEvwall),
//                new DriveToPointCommand(driveSubsystem, new Pose2d(35, -56, Rotation2d.fromDegrees(180)), 1, 5).withTimeout(200),
                //wait
                new WaitCommand(80),
               // set wrist to ready position for high chamber
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchEvAutoHighChamber, rollEvAutoHighChamber),

                //raise arm up





                // Drive to high chamber
                new ParallelCommandGroup(
                        new InstantCommand(() -> armSubsystem.setArm(24)),
                        new DriveToPointCommand(driveSubsystem, new Pose2d(highChamberEvRight.getX(), highChamberEvRight.getY(), Rotation2d.fromDegrees(38)),3, 5).withTimeout(1000)
                ),

                //wait then extend slides
                new ArmCoordinatesCommand(armSubsystem, armEvHighChamberX, armEvHighChamberY),
                new DriveToPointCommand(driveSubsystem, new Pose2d(highChamberEvRight.getX(), highChamberEvRight.getY(), Rotation2d.fromDegrees(39)),3, 5).withTimeout(1000),

//                new WaitForSlideCommand(armSubsystem, armEvHighChamberX, 10),
                new DriveToPointCommand(driveSubsystem, highChamberEvRight ,1, 5).withTimeout(500),
                //wait
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchEvAutoHighChamber, rollEvAutoHighChamber),
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollEvwall),
                new DriveToPointCommand(driveSubsystem, new Pose2d(30, -56, Rotation2d.fromDegrees(80)), 1, 3).withTimeout(100)



//                new WaitCommand(200),
                // Score specimen


        );

        addRequirements(armSubsystem, intakeSubsystem);
    }
}
