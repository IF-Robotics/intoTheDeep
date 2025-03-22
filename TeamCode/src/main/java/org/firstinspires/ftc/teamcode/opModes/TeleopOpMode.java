package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandGroups.ClimbLevel3;
import org.firstinspires.ftc.teamcode.commandGroups.DropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.DropOffCommand;
import org.firstinspires.ftc.teamcode.commandGroups.HighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeCloseCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterWallIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commandGroups.ScoreHighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.HighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.ArmManualCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ResetSlides;
import org.firstinspires.ftc.teamcode.commandGroups.TeleopSpecScore;

import org.firstinspires.ftc.teamcode.commands.VisionToSampleInterpolate;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForArmCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;


@Disabled
@TeleOp(name="teleOpFunnyTest")
public class TeleopOpMode extends Robot {



    //buttons
    private Button cross1, back2, start2, dUp1, dDown1, dLeft1, dRight1, bRight1, bLeft1, triangle1, triangle2, square1, touchpad1, touchpad2, start1, square2, dUp2, bRight2, bLeft2, dRight2, dDown2, cross2, circle1, circle2, dLeft2, back1;
    private Trigger tLeft1, tRight1, tLeft2, tRight2;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void initialize(){
        super.initialize();

        //configureMoreCommands();
        configureButtons();
        manualArm = false;
        manualSlides = false;

        new ArmCoordinatesCommand(armSubsystem, 3.5, 14).schedule(true);

    }

    /*public void configureMoreCommands() {

    }*/

    public void configureButtons() {
        square1 = new GamepadButton(m_driver, GamepadKeys.Button.X);
        square2 = new GamepadButton(m_driverOp, GamepadKeys.Button.X);
        start2 = new GamepadButton(m_driverOp, GamepadKeys.Button.START);
        back2 = new GamepadButton(m_driverOp, GamepadKeys.Button.BACK);
        dUp1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_UP);
        dUp2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_UP);
        dDown1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_DOWN);
        dDown2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_DOWN);
        dLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_LEFT);
        dLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_LEFT);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        bRight1 = new GamepadButton(m_driver, GamepadKeys.Button.RIGHT_BUMPER);
        bLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.LEFT_BUMPER);
        bRight2 = new GamepadButton(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        triangle1 = new GamepadButton(m_driver, GamepadKeys.Button.Y);
        triangle2 = new GamepadButton(m_driverOp, GamepadKeys.Button.Y);
        cross1 = new GamepadButton(m_driver, GamepadKeys.Button.A);
        cross2 = new GamepadButton(m_driverOp, GamepadKeys.Button.A);
        bLeft2 = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_BUMPER);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        dRight2 = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_RIGHT);
        tLeft1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        tRight1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1);
        tLeft2 = new Trigger(() -> m_driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        tRight2 = new Trigger(() -> m_driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1);
        start1 = new GamepadButton(m_driver, GamepadKeys.Button.START);
        circle1 = new GamepadButton(m_driver, GamepadKeys.Button.B);
        circle2 = new GamepadButton(m_driverOp, GamepadKeys.Button.B);
        back1 = new GamepadButton(m_driver, GamepadKeys.Button.BACK);





        //sub intake
        dUp1.whenPressed(new ConditionalCommand(
                new ConditionalCommand(
                        new IntakeSub(armSubsystem, intakeSubsystem),
                        new SequentialCommandGroup(
                                new WaitForArmCommand(armSubsystem, 0, 5, true),
                                new IntakeSub(armSubsystem, intakeSubsystem)
                        ),
                        () -> (armSubsystem.getArmAngle() < 5 && armSubsystem.getCurrentY() < 20)
                ),
                new RetractFromBasket(driveSubsystem, armSubsystem, intakeSubsystem),
                () -> armSubsystem.getCurrentY() < 30

                ));
        dUp2.whenPressed(new IntakeSub(armSubsystem, intakeSubsystem));



        //dUp2.whenReleased(armInSubCommand);
        //rotate intake
        bLeft1.whenActive(new InstantCommand(() -> intakeSubsystem.rotateIntake()));
        bLeft2.whenPressed(new InstantCommand(() -> intakeSubsystem.rotateIntake()));

        //intake close
        dRight1.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));
        dRight2.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));
        //retract after intaking
        dDown1.whenPressed(new ConditionalCommand(
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                        new SequentialCommandGroup(
                                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                                new HighBasketCommand(armSubsystem, intakeSubsystem)
                        ),
                        () -> teleopSpec == true
            )
        );

        dDown2.whenPressed(new RetractAfterIntake(armSubsystem, intakeSubsystem));
        //wall intake
//        tRight1.toggleWhenActive(new teleopSpecScore(driveSubsystem,armSubsystem,intakeSubsystem));
//        tLeft1.whenActive(new VisionToSampleInterpolate(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, false, ()->{return false;},m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX));
        tLeft2.whenActive(new ConditionalCommand(
                new ParallelCommandGroup(armWhenIntakeWallCommand, intakeWallCommand),
                new RetractAfterWallIntake(armSubsystem, intakeSubsystem),
                () -> {
                    armSubsystem.toggleWallState();
                    return armSubsystem.getWallState();
                }
        ));

        //chambers
        square2.whenPressed(new HighChamberCommand(armSubsystem, intakeSubsystem));
        square2.whenReleased(new ScoreHighChamberCommand(armSubsystem, intakeSubsystem));
        //auto spec scoring
        square1.toggleWhenPressed(new ConditionalCommand(
                new TeleopSpecScore(driveSubsystem,armSubsystem,intakeSubsystem),
                new ParallelCommandGroup(new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY), new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall)),
                () -> (armSubsystem.getTargetX() == armIntakeWallX && armSubsystem.getTargetY() == armIntakeWallY)
                )
        );

        //dropping sample (into observation zone)
        circle2.whenPressed(new DropCommand(armSubsystem, intakeSubsystem));
        circle2.whenReleased(dropOffCommandOp2);
        tRight1.whenActive(
                new ConditionalCommand(
                        new DropCommand(armSubsystem, intakeSubsystem),
                        new SequentialCommandGroup(
                                new WaitForSlideCommand(armSubsystem, 8, 20),
                                new WaitForArmCommand(armSubsystem, 0, 5)
                        ),
                        () -> armSubsystem.getArmAngle() < 45
                )
        );
        tRight1.whenInactive(new DropOffCommand(armSubsystem, intakeSubsystem));


        //baskets
        triangle2.whenPressed(new HighBasketCommand(armSubsystem, intakeSubsystem));
        triangle1.whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                    //move to high basket
                    new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                    new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket)
                        ),
                new SequentialCommandGroup(
                        new WaitForSlideCommand(armSubsystem, 8, 15),
                        //move arm back
                        new WaitForArmCommand(armSubsystem, 100, 45),

                        //move to high basket
                        new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY),
                        new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket)
                ),
                () -> armSubsystem.getArmAngle() > 45
                )
        );

        //retract after scoring in the baskets
        cross1.whenPressed(new SequentialCommandGroup(
                new RetractFromBasket(driveSubsystem, armSubsystem, intakeSubsystem),
                new IntakeSub(armSubsystem, intakeSubsystem).alongWith(new TeleDriveCommand(driveSubsystem, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX))
                )
        );
        cross2.whenPressed(new RetractFromBasket(driveSubsystem, armSubsystem, intakeSubsystem));

        //climbing
        bRight2.whenPressed(new ParallelCommandGroup(
                new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber),
                new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.DOWN)),
                armPositionToClimb));
        bRight2.whenReleased(new ClimbLevel3(armSubsystem, intakeSubsystem, gyro));

        //testing
        start1.whenPressed(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitch, roll));
        start2.whenPressed(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket));
        start2.whenPressed(new ArmManualCommand(armSubsystem, m_driverOp::getRightY, m_driverOp::getLeftY));

        //reset pinpoint imu
        back1.whenPressed(new InstantCommand(() -> driveSubsystem.resetPinpointIMU()));
        //reset slides
        tRight2.whenActive(new ResetSlides(armSubsystem));

        //Default Commands
        driveSubsystem.setDefaultCommand(teleDriveCommand);
    }

    @Override
    public void run(){
        super.run();


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        //retract and move arm out of the way
        if(gamepad1.right_stick_button || gamepad2.back){
            schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenBasket));
            schedule(new WaitForSlideCommand(armSubsystem, 8, 10));
            schedule(new InstantCommand(() -> armSubsystem.setArm(45)));
        }

        //extend after dropOff
        /*if(armSubsystem.getLastCommand().getName().equals("DropOffCommand") && Math.abs(driveSubsystem.getPos().getRotation().getDegrees()) < 100){
            schedule(new IntakeSub(armSubsystem, intakeSubsystem));
        }*/

        //switch teleop mode
        if(currentGamepad1.touchpad && !previousGamepad1.touchpad || currentGamepad2.touchpad && !previousGamepad2.touchpad){
            teleopSpec = !teleopSpec;
        }
    }


}