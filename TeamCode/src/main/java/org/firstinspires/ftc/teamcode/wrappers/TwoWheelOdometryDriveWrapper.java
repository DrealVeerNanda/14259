package org.firstinspires.ftc.teamcode.wrappers;


import org.firstinspires.ftc.teamcode.Async;

public class TwoWheelOdometryDriveWrapper {
    private static final double EPSILON_D = 0.02;
    private static final double EPSILON_R = 0.002;

    private MecanumWrapper mecanumWrapper;
    private TwoWheelOdometryWrapper odometryWrapper;
    private PIDWrapper xPIDWrapper;
    private PIDWrapper yPIDWrapper;
    private PIDWrapper rPIDWrapper;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetR = 0.0;

    public TwoWheelOdometryDriveWrapper() {
        this.xPIDWrapper = new PIDWrapper()
                .setErrorFunction(() -> {
                    return targetX - this.odometryWrapper.getX();
                })
                .setResponseFunction(factor -> {
                    double r = this.odometryWrapper.getR();
                    double deltaX = factor * Math.cos(r);
                    double deltaY = factor * Math.sin(r);
                    this.mecanumWrapper.addPowerX(deltaX);
                    this.mecanumWrapper.addPowerY(deltaY);
                });

        this.yPIDWrapper = new PIDWrapper()
                .setErrorFunction(() -> {
                    return targetY - this.odometryWrapper.getY();
                })
                .setResponseFunction(factor -> {
                    double r = this.odometryWrapper.getR() + Math.PI / 2.0;
                    double deltaX = -factor * Math.cos(r);
                    double deltaY = -factor * Math.sin(r);
                    this.mecanumWrapper.addPowerX(deltaX);
                    this.mecanumWrapper.addPowerY(deltaY); 
                });

        this.rPIDWrapper = new PIDWrapper()
                .setErrorFunction(() -> {
                    return targetR - this.odometryWrapper.getR();
                })
                .setResponseFunction(factor -> {
                    this.mecanumWrapper.addPowerR(-factor);
                });
    }

    public TwoWheelOdometryDriveWrapper setMecanumWrapper(MecanumWrapper mecanumWrapper) {
        this.mecanumWrapper = mecanumWrapper;
        return this;
    }

    public TwoWheelOdometryDriveWrapper setOdometryWrapper(TwoWheelOdometryWrapper odometryWrapper) {
        this.odometryWrapper = odometryWrapper;
        return this;
    }

    public TwoWheelOdometryDriveWrapper setPose(double targetX, double targetY, double targetR) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetR = targetR;
        return this;
    }

    public boolean isBusy() {
        double deltaX = this.odometryWrapper.getX() - targetX;
        double deltaY = this.odometryWrapper.getY() - targetY;
        double deltaR = this.odometryWrapper.getR() - targetR;
        return deltaX * deltaX + deltaY * deltaY > TwoWheelOdometryDriveWrapper.EPSILON_D ||
                deltaR * deltaR > TwoWheelOdometryDriveWrapper.EPSILON_R;
    }

    public MecanumWrapper getMecanumWrapper() {
        return this.mecanumWrapper;
    }

    public TwoWheelOdometryWrapper getOdometryWrapper() {
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
