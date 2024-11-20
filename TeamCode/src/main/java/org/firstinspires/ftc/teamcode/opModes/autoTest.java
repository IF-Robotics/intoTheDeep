package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.testX;
import static org.firstinspires.ftc.teamcode.other.Globals.testY;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
@Config
@Autonomous(name="autoTest")
public class autoTest extends Robot {

    private Button dUp1, dDown1, dLeft1, dRight1;

    public static double sprintDistance = 20;

    @Override
    public void initialize(){
        super.initialize();



        dUp1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_UP);
        dDown1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_DOWN);
        dLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_LEFT);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);

        dUp1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(0, sprintDistance, new Rotation2d(0)), 0, 0, 1000000));
        dDown1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(0, -sprintDistance, new Rotation2d(0)), 0, 0, 1000000));
        dLeft1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(-sprintDistance, 0, new Rotation2d(0)), 0, 0, 1000000));
        dRight1.whenPressed(new DriveToPointCommand(driveSubsystem, new Pose2d(sprintDistance, 0, new Rotation2d(0)), 0, 0, 1000000));


        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.setStartingPos(new Pose2d(0, 0, Rotation2d.fromDegrees(0))))
        ));

        driveSubsystem.setDefaultCommand(new DriveToPointCommand(driveSubsystem, new Pose2d(testX, testY, new Rotation2d(0)) ,0, 0,1000000));

    }

}
