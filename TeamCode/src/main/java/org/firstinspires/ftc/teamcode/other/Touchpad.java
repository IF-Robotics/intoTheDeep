package org.firstinspires.ftc.teamcode.other;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.button.Button;

public class Touchpad extends Button {
    @Override
    public boolean get() {
        if(gamepad1.touchpad){
            return true;
        } else {
            return false;
        }
    }
}