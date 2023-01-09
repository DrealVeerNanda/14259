package org.firstinspires.ftc.teamcode.wrappers;

import org.firstinspires.ftc.teamcode.Async;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class DriveWrapper{
    private static final double EPSILON = 0.1;
    private static final double WEIGHT_X = 0.02;
    private static final double WEIGHT_Y = 0.02;
    private static final double WEIGHT_R = 0.5;

    private MecanumWrapper mecanumWrapper;
    private OdometryWrapper odometryWrapper;
    private PIDWrapper xPIDWrapper;
    private PIDWrapper yPIDWrapper;
    private PIDWrapper rPIDWrapper;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetR = 0.0;

    public DriveWrapper() {
        this.xPIDWrapper = new PIDWrapper()
                .setErrorFunction(() -> {
                    return targetX - this.odometryWrapper.getX();
                })
                .setResponseFunction(factor -> {
                    this.mecanumWrapper.setPowerX(factor);
                });

        this.yPIDWrapper = new PIDWrapper()
                .setErrorFunction(() -> {
                    return targetY - this.odometryWrapper.getY();
                })
                .setResponseFunction(factor -> {
                    this.mecanumWrapper.setPowerY(factor);
                });

        this.rPIDWrapper = new PIDWrapper()
                .setErrorFunction(() -> {
                    return targetR - this.odometryWrapper.getR();
                })
                .setResponseFunction(factor -> {
                    this.mecanumWrapper.setPowerR(factor);
                });
    }

    public DriveWrapper setMecanumWrapper(MecanumWrapper mecanumWrapper) {
        this.mecanumWrapper = mecanumWrapper;
        return this;
    }

    public DriveWrapper setOdometryWrapper(OdometryWrapper odometryWrapper) {
        this.odometryWrapper = odometryWrapper;
        return this;
    }

    public DriveWrapper setPose(double targetX, double targetY, double targetR) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetR = targetR;
        return this;
    }

    public Async.AsyncBody gotoPose(double targetX, double targetY, double targetR) {
        return async -> {
            this.setPose(targetX, targetY, targetR);
            this.odometryWrapper.subscribePoseEvent((currentX, currentY, currentR) -> {
                double deltaX = (currentX - targetX) * DriveWrapper.WEIGHT_X;
                double deltaY = (currentY - targetY) * DriveWrapper.WEIGHT_Y;
                double deltaR = (currentR - targetR) * DriveWrapper.WEIGHT_R;
                if(deltaX * deltaX + deltaY * deltaY + deltaR * deltaR > DriveWrapper.EPSILON) return false;
                async.finish();
                return true;
            });
        };
    }

    public MecanumWrapper getMecanumWrapper() {
        return this.mecanumWrapper;
    }

    public OdometryWrapper getOdometryWrapper() {
        return this.odometryWrapper;
    }

    public double getTargetX() {
        return this.targetX;
    }

    public double getTargetY() {
        return this.targetY;
    }

    public double getTargetR() {
        return this.targetR;
    }

    public PIDWrapper getXPIDWrapper() {
        return this.xPIDWrapper;
    }

    public PIDWrapper getYPIDWrapper() {
        return this.yPIDWrapper;
    }

    public PIDWrapper getRPIDWrapper() {
        return this.rPIDWrapper;
    }

    private void updateX() {
        this.xPIDWrapper.update();
    }

    private void updateY() {
        this.yPIDWrapper.update();
    }

    private void updateR() {
        this.rPIDWrapper.update();
    }

    private void updateMecanum() {
        this.mecanumWrapper.update();
    }

    private void updateOdometry() {
        this.odometryWrapper.update();
    }

    public void update() {
        this.updateOdometry();
        this.updateX();
        this.updateY();
        this.updateR();
        this.updateMecanum();
    }
}
