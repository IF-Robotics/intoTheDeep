package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
//intake subsystem
    //intaking
    public static int pitchWhenIntake = 0;
    //public static int rollWhenReadyIntake = -200;
    public static int rollWhenIntake = -190;
    //public static int rollWhenCloseIntake = -200;

    //intake power
    public static double clawOpen = .75;
    public static double clawClose = .5;

    //scoring
    public static int pitchWhenBasket = 0;
    public static int rollWhenBasket = 150;
    public static double pitchWhenHighChamber = -100;
    public static double rollWhenHighChamber = -20;
    public static double pitchFrontHighChamber = 70;
    public static double rollFrontHighChamber = 140;

    //home
    public static int rollWhenArmBack = -150;
    public static int rollWhenArmHome = 200;

//arm subsystem
    //arm home
    public static double armHomeX = 7.5;
    public static double armHomeY = 7;
    //arm back
    public static double armBackX = -3;
    public static double armBackY = 16;
    //arm high basket
    public static double armHighBasketX = 0;
    public static double  armHighBasketY = 48;
    //arm when intaking from sub
    public static double armReadySubIntakeX = 26;
    public static double armReadySubIntakeY = 10.5;
    public static double armSubIntakeY = 7;
    //arm when close (distance) intake
    public static double armCloseIntakeX = 15;
    public static double armCloseIntakeY = 7;
    //arm when front high chamber
    public static double armFrontHighChamberX = 24;
    public static double armFrontHighChamberY = 19.31;
    //arm when high chamber
    public static double armHighChamberX = -3;
    public static double armHighChamberY = 20.5;
    //manual arm boolean
    public static boolean manualArm = false;


//drive to point
    public static double translationKP = 0.015;
    public static double translationKI = 0.0;
    public static double translationKD = 0.0005;
    public static double headingKP = 0.008;
    public static double headingKI = 0.0;
    public static double headingKD = 0.0001;
    public static double lateralMutliplier = 1.5;

    public static double testX = 0.0;
    public static double testY = 0.0;
    public static double testHeading = 0.0;

}
