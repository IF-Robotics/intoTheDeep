package org.firstinspires.ftc.teamcode.other;

public abstract class AutoBase extends Robot{

    public void initialize() {
        super.initialize();

        //reset slide encoder
        slideLeft.resetEncoder();
    }

    public void configureAutoCommands(){

    }
}
