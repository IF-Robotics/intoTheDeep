package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.other.Touchpad;

@TeleOp(name="teleOpFunnyTest")
public class TeleopOpMode extends Robot {

    //private

    //buttons
    private Button cross1, back2, start2, dUp1, dDown1, dLeft1, dRight1, bRight1, bLeft1, triangle1, square1, touchpad1, start1;
    private Trigger tLeft1, tRight1;


    @Override
    public void initialize(){
        super.initialize();

        //configureMoreCommands();
        configureButtons();
    }

    /*public void configureMoreCommands() {

    }*/

    public void configureButtons() {
        square1 = new GamepadButton(m_driver, GamepadKeys.Button.X);
        start2 = new GamepadButton(m_driverOp, GamepadKeys.Button.START);
        back2 = new GamepadButton(m_driverOp, GamepadKeys.Button.BACK);
        dUp1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_UP);
        dDown1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_DOWN);
        dLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_LEFT);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        bRight1 = new GamepadButton(m_driver, GamepadKeys.Button.RIGHT_BUMPER);
        bLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.LEFT_BUMPER);
        triangle1 = new GamepadButton(m_driver, GamepadKeys.Button.Y);
        cross1 = new GamepadButton(m_driver, GamepadKeys.Button.A);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        touchpad1 = new Touchpad();
        tLeft1 = new Trigger(() -> m_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1);
        start1 = new GamepadButton(m_driver, GamepadKeys.Button.START);

        //sub intake
        dUp1.whenPressed(armWhenIntakeCommand);
        dUp1.whenPressed(intakeReadyCommand);
        bLeft1.whenPressed(intakeCommand);
        bLeft1.whenReleased(intakeReadyCommand);
        tLeft1.whenActive(outakeReadyCommand);
        //intake close
        dRight1.whenPressed(armWhenCloseIntakeCommand);
        dRight1.whenPressed(intakeCloseCommand);
        //retract after intaking
        dDown1.whenPressed(new RetractAfterIntake(armSubsystem, intakeSubsystem));

        //chambers
        square1.whenPressed(armWhenHighChamberCommand);
        square1.whenPressed(intakeWhenHighChamberCommand);

        //baskets
        triangle1.whenPressed(armHighBasketCommand);
        triangle1.whenPressed(intakeWhenHighBasketCommand);

        //retract after scoring in the baskets
        cross1.whenPressed(new RetractFromBasket(armSubsystem, intakeSubsystem));

        //arm back
        dLeft1.whenPressed(armBackCommand);
        dLeft1.whenPressed(intakeWhenArmBackCommand);

        //climbing
        start2.whenPressed(intakeWhenHighBasketCommand);
        start2.whenPressed(armManualCommand);

        //testing
        start1.whenPressed(setIntakeCommand);

        //Default Commands
        driveSubsystem.setDefaultCommand(teleDriveCommand);
    }
}