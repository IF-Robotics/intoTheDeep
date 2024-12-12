package org.firstinspires.ftc.teamcode.commandGroups;

import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftBasketPose;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideLeftSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideMidSpike;
import static org.firstinspires.ftc.teamcode.other.PosGlobals.leftSideRightSpike;
import static org.firstinspires.ftc.teamcode.other.Robot.*;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.commands.holdDTPosCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public class CycleLeftSpikeMarks extends SequentialCommandGroup {

    public CycleLeftSpikeMarks(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                //drive to first sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideRightSpike,2, 5),
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(400),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(400),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(500),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(500),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to second sample on the spikemark
                new DriveToPointCommand(driveSubsystem, leftSideMidSpike,2, 5),
                //wait
                new WaitCommand(200),
//                new DriveToPointCommand(driveSubsytem, )
                intakeCloseCommand,
                armWhenCloseIntakeCommand,
                new WaitCommand(300),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(400),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(500),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(300),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(300),
                //reach out the arm and intake
                intakeCloseCommand,
                armWhenCloseIntakeCommand,


                //drive to third sample on the spikemark
                intakeLastLeftAutoCommand,
                new DriveToPointCommand(driveSubsystem, leftSideLeftSpike,2, 5),
                armWhenCloseIntakeCommand,
                new WaitCommand(500),
                //grab and retract
                retractAfterIntake,
                new WaitCommand(500),
                //arm & intake to high basket
                armHighBasketCommand,
                intakeWhenHighBasketCommand,
                new WaitCommand(500),
                //drive to high basket
                new DriveToPointCommand(driveSubsystem, leftBasketPose, 2, 5),
                //wait
                new WaitCommand(500),
                //drop sample & arm down
                retractFromBasket,
                new WaitCommand(500)
        );

        addRequirements(intakeSubsystem, armSubsystem);
    }
}
