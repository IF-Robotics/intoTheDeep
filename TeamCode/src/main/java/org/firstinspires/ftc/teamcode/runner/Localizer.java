package org.firstinspires.ftc.teamcode.runner;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public interface Localizer {
    Twist2dDual<Time> update();
}
