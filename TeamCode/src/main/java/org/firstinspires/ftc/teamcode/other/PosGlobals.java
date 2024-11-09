package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PosGlobals {
    //starting pos
    public static double startLeftX = 0;
    public static double startY = 0;
    public static double startRightX = -startLeftX;

    //high chamber
    public static double highRungLeftX = 0;
    public static double highRungY = 0;
    public static double highRungRightX = -highRungLeftX;

    public static double[] chamberXPoses = {};

    //baskets
    public static double basketX = 0;
    public static double basketY = 0;
    public static double basketHeading = 0;

    //observation zone pickup
    public static double obsZoneX = 0;
    public static double obsZoneY = 0;
    public static double obsZoneHeading = 0;
}
