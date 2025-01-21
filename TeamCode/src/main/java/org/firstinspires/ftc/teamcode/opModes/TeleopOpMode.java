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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandGroups.Climb;
import org.firstinspires.ftc.teamcode.commandGroups.ClimbLevel3;
import org.firstinspires.ftc.teamcode.commandGroups.DropCommand;
import org.firstinspires.ftc.teamcode.commandGroups.HighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeCloseCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterWallIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commandGroups.ScoreHighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.highBasketCommand;
import org.firstinspires.ftc.teamcode.commandGroups.scoreHighBasket;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ResetSlides;
import org.firstinspires.ftc.teamcode.commands.VisionClawCommand;
import org.firstinspires.ftc.teamcode.commands.VisionToSample;
import org.firstinspires.ftc.teamcode.commandGroups.teleopSpecScore;

import org.firstinspires.ftc.teamcode.commands.ResetSlides;
import org.firstinspires.ftc.teamcode.commands.WaitForSlideCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

@TeleOp(name="teleOpFunnyTest")
public class TeleopOpMode extends Robot {

    //private

    //buttons
    private Button cross1, back2, start2, dUp1, dDown1, dLeft1, dRight1, bRight1, bLeft1, triangle1, triangle2, square1, touchpad1, touchpad2, start1, square2, dUp2, bRight2, bLeft2, dRight2, dDown2, cross2, circle1, circle2, dLeft2, back1;
    private Trigger tLeft1, tRight1, tLeft2, tRight2;


    @Override
    public void initialize(){
        super.initialize();

        //configureMoreCommands();
        configureButtons();
        manualArm = false;

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
                new IntakeSub(armSubsystem, intakeSubsystem),
                new RetractFromBasket(armSubsystem, intakeSubsystem),
                () -> armSubsystem.getCurrentY() < 20
                ));
        dUp2.whenPressed(new IntakeSub(armSubsystem, intakeSubsystem));
        dUp2.whenReleased(armInSubCommand);
        //rotate intake
        bLeft1.whenPressed(new InstantCommand(() -> intakeSubsystem.rotateIntake()));
        bLeft2.whenPressed(new InstantCommand(() -> intakeSubsystem.rotateIntake()));
//        bLeft1.whileActiveContinuous(new VisionClawCommand(intakeSubsystem, visionSubsystem));
//        bLeft2.whileActiveContinuous(new VisionClawCommand(intakeSubsystem, visionSubsystem));

        //intake close
        dRight1.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));
        dRight2.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));
        //retract after intaking
        dDown1.whenPressed(new SequentialCommandGroup(
                new RetractAfterIntake(armSubsystem, intakeSubsystem),
                new highBasketCommand(armSubsystem, intakeSubsystem)
                )
        );

        dDown2.whenPressed(new RetractAfterIntake(armSubsystem, intakeSubsystem));
        //wall intake
//        tRight1.toggleWhenActive(new teleopSpecScore(driveSubsystem,armSubsystem,intakeSubsystem));
        tRight1.whenActive(new VisionToSample(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, ()->{return false;},m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX));
        tLeft2.whenActive(new ConditionalCommand(
                new ParallelCommandGroup(armWhenIntakeWallCommand, intakeWallCommand),
                retractAfterWallIntake,
                () -> {
                    armSubsystem.toggleWallState();
                    return armSubsystem.getWallState();
                }
        ));


        //auto align
//        tRight1.whenActive(new VisionClawCommand(intakeSubsystem, visionSubsystem), true);
//        tRight2.whenActive(new VisionClawCommand(intakeSubsystem, visionSubsystem), true);

        //chambers
        square1.whenPressed(new HighChamberCommand(armSubsystem, intakeSubsystem));
        square1.whenReleased(new ScoreHighChamberCommand(armSubsystem, intakeSubsystem));
        square2.whenPressed(new HighChamberCommand(armSubsystem, intakeSubsystem));
        square2.whenReleased(new ScoreHighChamberCommand(armSubsystem, intakeSubsystem));

        //dropping sample (into observation zone)
        circle2.whenPressed(dropCommand);
        circle2.whenReleased(new SequentialCommandGroup(
                new InstantCommand(() -> intakeSubsystem.openClaw()),
                new WaitCommand(50),
                new InstantCommand(() -> armSubsystem.setSlide(8))
                )
        );


        //baskets
        triangle1.whenPressed(new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY)/*armHighBasketCommand*/);
        triangle1.whenPressed(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollWhenBasket)/*intakeWhenHighBasketCommand*/);
        triangle2.whenPressed(new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY)/*armHighBasketCommand*/);
        triangle2.whenPressed(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchIntakeWall, rollWhenBasket)/*intakeWhenHighBasketCommand*/);

        //retract after scoring in the baskets
        cross1.whenPressed(new SequentialCommandGroup(
                new RetractFromBasket(armSubsystem, intakeSubsystem),
                new IntakeSub(armSubsystem, intakeSubsystem)
                )
        );
        cross2.whenPressed(new RetractFromBasket(armSubsystem, intakeSubsystem));

        //climbing
        bRight2.whenPressed(new ParallelCommandGroup(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber),armPositionToClimb));
        bRight2.whenReleased(new ClimbLevel3(armSubsystem, intakeSubsystem, gyro));

        //auto scoring
//        tLeft1.whenActive(new scoreHighBasket(driveSubsystem, armSubsystem, intakeSubsystem));
        //testing
        start1.whenPressed(setIntakeCommand);
        start2.whenPressed(intakeWhenHighBasketCommand);
        start2.whenPressed(armManualCommand);

        //reset pinpoint imu
        back1.whenPressed(new InstantCommand(() -> driveSubsystem.resetPinpointIMU()));
        //reset slides
        tRight2.whenActive(new ResetSlides(armSubsystem));

        //Default Commands
        driveSubsystem.setDefaultCommand(teleDriveCommand);
    }

    /*@Override
    public void run(){

        //retract
        if(gamepad1.right_stick_button || gamepad2.right_stick_button){
            schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 0));
            schedule(new WaitForSlideCommand(armSubsystem, 8, 10));
        }
    }
    */

}