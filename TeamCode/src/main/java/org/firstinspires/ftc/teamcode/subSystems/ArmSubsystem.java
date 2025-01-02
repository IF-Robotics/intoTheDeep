package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;

@Config
public class ArmSubsystem extends SubsystemBase {

    private MotorEx arm, slideL;
    private MotorGroup slide;
    private Servo endStop;
    private AnalogInput armEncoder;
    private Telemetry telemetry;


    //arm PIDF
    public static double kParm = 0.05, kIarm = 0, kDarm = 0.01, kFarm = .3;
    public static double armWeakKP = 0.03;
    public static double armAngleOffset = -208.5/*-39*/;
    public static double armSuperWeakKP = .007;
    private double ff;
    private PIDController armController;
    private double setArmTargetAngle = 0;
    private double armPower;
    private double rawAngle;
    private double correctedAngle = 0;

    //slide pidf
    public static double slideKP = .4, slideKI = 0.0, slideKD = 0.0, slideKF = 0;
    private PIDController slideController;
    private final double ticksPerIn = 2786/32.75;
    private int slideTicks = 1;
    private double slidePower = 0;
    private double slideExtention = 0;
    public static double slideWristOffset = 7.75;
    private double setSlideTarget = 7.75;
    private double slideError = 0;

    //arm coordinates
    private double slideTargetIn;
    private double armTargetAngle;
    private double armHeight = (24.5 + 24 + 6*24) / 25.4;
    private double targetX = armFoldX;
    private double targetY = armFoldY;

    //manualArm
    private double armManualPower;
    private double slideManualPower;

    //intakeFromWall
    private boolean wallActive;

    //slide velocity
    private LinkedList<TimeStampedPosition> positionHistory = new LinkedList<>();
    private static final long VELOCITY_TIME_FRAME_MS = 50; // Time frame in milliseconds

    //endstop
    private Endstop endstop;
    public enum Endstop{
        UP,
        DOWN
    }

    //constructor
    public ArmSubsystem(MotorEx arm, MotorEx slideL, MotorGroup slide, Servo endStop, AnalogInput armEncoder, Telemetry telemetry) {
        this.arm = arm;
        this.slide = slide;
        this.slideL = slideL;
        this.endStop = endStop;
        this.armEncoder = armEncoder;
        this.telemetry = telemetry;
        wallActive = false;
    }

    public void manualArm(double armPower, double slidePower){
        armManualPower = -armPower;
        slideManualPower = slidePower;
    }

    public void setArm(double targetAngle) {
        setArmTargetAngle = targetAngle;
    }

    public void setSlide(double targetInches) {
        setSlideTarget = targetInches;
    }

    public void setArmCoordinates(double x, double y){
        targetX = x;
        targetY = y;

        //inverse kinematics
        slideTargetIn = Math.sqrt(Math.pow(x, 2) + Math.pow(y - armHeight, 2));
        armTargetAngle = Math.toDegrees(Math.atan2((y - armHeight), x));

        //write
        setArm(armTargetAngle);
        setSlide(slideTargetIn);
    }

    public void setArmY(double y){
        targetY = y;
        setArmCoordinates(targetX, y);
    }

    public void setArmX(double x){
        targetX = x;
        setArmCoordinates(x, targetY);
    }

    public void setEndstop(Endstop endstop){
        if(endstop == Endstop.UP){
            endStop.setPosition(endstopUp);
        } else if (endstop == Endstop.DOWN){
            endStop.setPosition(endstopDown);
        }
    }

    public void setArmP(double p){
        armController.setP(p);
    }

    //forward kinematics
    public double getCurrentX(){
        return slideExtention * Math.cos(Math.toRadians(correctedAngle));
    }
    public double getCurrentY(){
        return slideExtention * Math.sin(Math.toRadians(correctedAngle)) + armHeight;
    }

    public double getArmAngle(){
        return correctedAngle;
    }

    public double getSlideExtention(){
        return slideExtention;
    }

    public double getSlideError(){
        return slideError;
    }

    public double getArmTarget(){
        return setArmTargetAngle;
    }

    public double getSlideTarget(){
        return setSlideTarget;
    }

    //return intakeWall state
    public boolean getWallState(){
        return wallActive;
    }
    //toggle wall state
    public void toggleWallState(){
        wallActive = !wallActive;
    }


    //slide velocity
    public double getSlideVelocity() {

        long currentTime = System.currentTimeMillis();

        // Add the new position with its timestamp
        positionHistory.add(new TimeStampedPosition(getSlideExtention(), currentTime));

        // Remove old entries beyond the time frame
        while (!positionHistory.isEmpty() &&
                currentTime - positionHistory.getFirst().getTimestamp() > VELOCITY_TIME_FRAME_MS) {
            positionHistory.removeFirst();
        }


        if (positionHistory.size() < 2) {
            // Not enough data to calculate velocity
            return 0.0;
        }

        // Get the oldest and newest positions in the time frame
        TimeStampedPosition oldestPosition = positionHistory.getFirst();
        TimeStampedPosition newestPosition = positionHistory.getLast();

        // Calculate velocity: Δposition / Δtime
        double deltaPosition = newestPosition.getPosition() - oldestPosition.getPosition();
        double deltaTime = (newestPosition.getTimestamp() - oldestPosition.getTimestamp()) / 1000.0; // Convert ms to seconds

        // Avoid divide-by-zero errors
        if (deltaTime == 0) {
            return 0.0;
        }

        return deltaPosition / deltaTime; // Velocity in INCHES per second
    }

    public class TimeStampedPosition {
        private final double position; // For the arm, could be degrees or extension length
        private final long timestamp;  // Timestamp in milliseconds

        public TimeStampedPosition(double position, long timestamp) {
            this.position = position;
            this.timestamp = timestamp;
        }

        public double getPosition() {
            return position;
        }

        public long getTimestamp() {
            return timestamp;
        }

        @Override
        public String toString() {
            return "TimeStampedPosition{" +
                    "position=" + position +
                    ", timestamp=" + timestamp +
                    '}';
        }
    }

    @Override
    public void periodic() {
        //read
        slideTicks = slideL.getCurrentPosition();
        rawAngle = armEncoder.getVoltage()/3.3 * 360;
        correctedAngle = rawAngle + armAngleOffset;

        //arm pid
        //lowering the kp when the arm is up
        if(correctedAngle > 80){
            armController = new PIDController(armWeakKP, kIarm, kDarm);
        } else {
            armController = new PIDController(kParm, kIarm, kDarm);
        }
        //feed forward
        ff = kFarm * Math.cos(Math.toRadians(correctedAngle));
        armPower = armController.calculate(correctedAngle, setArmTargetAngle) + ff;
        /*telemetry.addData("ff", ff);
        telemetry.addData("cos", Math.cos(Math.toRadians(angle)));;
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("error", targetAngle - angle);*/

        //slide pid
        slideController = new PIDController(slideKP, slideKI, slideKD);
        slidePower = slideController.calculate(slideExtention, setSlideTarget) + slideKF;
        //telemetry.addData("targetIN", targetInches);
        //telemetry.addData("slideTicks", slideTicks);
        slideError = setSlideTarget - slideExtention;
        telemetry.addData("slideError", slideError);

        //arm manual
        if(manualArm){
            arm.set(armManualPower);
            slide.set(slideManualPower);
        } else {
            //pid power
            arm.set(armPower);
            slide.set(slidePower);
        }

        //calculate slide extension
        slideExtention = (slideTicks/ticksPerIn + slideWristOffset);

        telemetry.addData("armAngle", correctedAngle);
        telemetry.addData("armPower", armPower);
        telemetry.addData("armKP", armController.getP());
        telemetry.addData("armError", setArmTargetAngle - correctedAngle);
        telemetry.addData("armAngleError", setArmTargetAngle - correctedAngle);

        telemetry.addData("slidePower", slidePower);
        telemetry.addData("slideExtention", slideExtention);

        telemetry.addData("xArmPos", getCurrentX());
        telemetry.addData("yArmPos", getCurrentY());
        telemetry.addData("slideVelocity", getSlideVelocity());

    }

    public void setPowerZero(){
        arm.set(0);
    }

}
