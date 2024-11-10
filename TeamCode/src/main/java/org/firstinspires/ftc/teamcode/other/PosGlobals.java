package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class PosGlobals {
    //starting pos
    public static final double startRightX = (3 + 7/16) + 14.74/2;//the 3&7/16 is the length a sample
    public static final double startY = -70.9 + (8.18898);
    public static Pose2d startingPosRight = new Pose2d(startRightX, startY, Rotation2d.fromDegrees(-180));
    public static Pose2d startingPosLeft = startingPosRight.plus(new Transform2d(new Translation2d(-2 * startRightX, 0), Rotation2d.fromDegrees(180)));
    public static double startLeftX = -startRightX;

    //high chamber
    public static double highChamberRightX = 8;
    public static Pose2d highChamberRight = new Pose2d(highChamberRightX, -40, Rotation2d.fromDegrees(180));
    public static Pose2d highChamberLeft = highChamberRight.plus(new Transform2d(new Translation2d(-2 * highChamberRightX, 0), Rotation2d.fromDegrees(180)));

    public static double[] chamberXPoses = {};

    //baskets
    public static Pose2d basketPose = new Pose2d(-50, -50, Rotation2d.fromDegrees(135));

    //observation zone pickup
    public static double obsZoneX = 0;
    public static double obsZoneY = 0;
    public static double obsZoneHeading = 0;
}
