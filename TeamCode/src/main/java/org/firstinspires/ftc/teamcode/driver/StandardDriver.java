package org.firstinspires.ftc.teamcode.driver;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class StandardDriver {
    private static Pid.Coefficients xPidCoefficients = new Pid.Coefficients(0.08, 0.1, 0.01);
    private static Pid.Coefficients yPidCoefficients = new Pid.Coefficients(0.08, 0.1, 0.01);
    private static Pid.Coefficients headingPidCoefficients = new Pid.Coefficients(0.7, 0.55, 0.01);

    private SampleMecanumDrive drive;
    private Pid xPid, yPid, headingPid;
    private double x = 0.0, y = 0.0, heading = 0.0;
    private double xPred = 0.0, yPred = 0.0, headingPred = 0.0, headingPredRaw = 0.0;
    private double xPower = 0.0, yPower = 0.0, headingPower = 0.0;

    public StandardDriver(HardwareMap hardwareMap) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.xPid = new Pid(
                StandardDriver.xPidCoefficients,
                () -> this.x - this.xPred,
                factor -> this.addPower(factor, -this.headingPred));
        this.yPid = new Pid(
                StandardDriver.yPidCoefficients,
                () -> this.y - this.yPred,
                factor -> this.addPower(factor, -this.headingPred + Math.PI / 2.0));
        this.headingPid = new Pid(
                StandardDriver.headingPidCoefficients,
                () -> this.heading - this.headingPred,
                factor -> this.addHeadingPower(factor));
    }

    private void addPower(double power, double r) {
        this.xPower += power * Math.cos(r);
        this.yPower += power * Math.sin(r);
    }

    private void addHeadingPower(double power) {
        this.headingPower += power;
    }

    public void addX(double x) { this.x += x; }
    public void addY(double y) { this.y += y; }
    public void addHeading(double heading) { this.heading += heading; }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }
    public void setHeading(double heading) { this.heading = heading; }

    public void update() {
        this.drive.update();
        Pose2d poseEstimate = this.drive.getPoseEstimate();
        this.xPred = poseEstimate.getX();
        this.yPred = poseEstimate.getY();
        double headingPredRaw = poseEstimate.getHeading();
        this.headingPred += Angle.normDelta(headingPredRaw - this.headingPredRaw);
        this.headingPredRaw = headingPredRaw;

        this.xPid.update();
        this.yPid.update();
        this.headingPid.update();

        this.drive.setWeightedDrivePower(new Pose2d(xPower, yPower, headingPower));
        this.drive.update();

        this.xPower = 0.0;
        this.yPower = 0.0;
        this.headingPower = 0.0;
    }
}
