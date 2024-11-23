package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandGroups.Climb;
import org.firstinspires.ftc.teamcode.commandGroups.HighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeCloseCommand;
import org.firstinspires.ftc.teamcode.commandGroups.IntakeSub;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterWallIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.other.Touchpad1;
@TeleOp(name="teleOpFunnyTest")
public class TeleopOpMode extends Robot {

    //private

    //buttons
    private Button cross1, back2, start2, dUp1, dDown1, dLeft1, dRight1, bRight1, bLeft1, triangle1, triangle2, square1, touchpad1, start1, square2, dUp2, bRight2, bLeft2, dRight2, dDown2, cross2;
    private Trigger tLeft1, tRight1;


    @Override
    public void initialize(){
        super.initialize();

        //configureMoreCommands();
        configureButtons();
        manualArm = false;

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
        touchpad1 = new Touchpad1();
        tLeft1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        tRight1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1);
        start1 = new GamepadButton(m_driver, GamepadKeys.Button.START);




        //sub intake
        dUp1.whenPressed(new IntakeSub(armSubsystem, intakeSubsystem));
        dUp2.whenPressed(new IntakeSub(armSubsystem, intakeSubsystem));
        dUp2.whenReleased(armInSubCommand);
        //rotate intake
        bLeft1.whenPressed(new InstantCommand(() -> intakeSubsystem.rotateIntake()));
        bLeft2.whenPressed(new InstantCommand(() -> intakeSubsystem.rotateIntake()));

        //intake close
        dRight1.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));
        dRight2.whenPressed(new IntakeCloseCommand(armSubsystem, intakeSubsystem));
        //retract after intaking
        dDown1.whenPressed(new RetractAfterIntake(armSubsystem, intakeSubsystem));
        dDown2.whenPressed(new RetractAfterIntake(armSubsystem, intakeSubsystem));
        //wall intake
        dLeft1.whenPressed(new ConditionalCommand(
                new ParallelCommandGroup(armWhenIntakeWallCommand, intakeWallCommand),
                new RetractAfterWallIntake(armSubsystem, intakeSubsystem),
                () -> {
                    armSubsystem.toggleWallState();
                    return armSubsystem.getWallState();
                }
        ));

        //chambers
        square1.whenPressed(new HighChamberCommand(armSubsystem, intakeSubsystem));

        //baskets
        triangle1.whenPressed(armHighBasketCommand);
        triangle1.whenPressed(intakeWhenHighBasketCommand);
        triangle2.whenPressed(armHighBasketCommand);
        triangle2.whenPressed(intakeWhenHighBasketCommand);

        //retract after scoring in the baskets
        cross1.whenPressed(new RetractFromBasket(armSubsystem, intakeSubsystem));
        cross2.whenPressed(new RetractFromBasket(armSubsystem, intakeSubsystem));

        //climbing
        bRight2.whenPressed(armPositionToClimb);
        bRight2.whenReleased(new Climb(armSubsystem, intakeSubsystem));

        //testing
        start1.whenPressed(setIntakeCommand);
        start2.whenPressed(intakeWhenHighBasketCommand);
        start2.whenPressed(armManualCommand);

        //Default Commands
        driveSubsystem.setDefaultCommand(teleDriveCommand);
    }
}