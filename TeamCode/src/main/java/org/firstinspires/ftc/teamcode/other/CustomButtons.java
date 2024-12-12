package org.firstinspires.ftc.teamcode.other;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.other.Robot.CustomButton.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CustomButtons extends Button {

    Gamepad gamepad;

    public CustomButtons(Gamepad gamepad, Robot.CustomButton... buttons){
        this.gamepad = gamepad;
    }

    @Override
    public boolean get() {
        switch(Robot.CustomButton){
            case TOUCH:
                return gamepad.touchpad;
        }
        return false;
    }
}