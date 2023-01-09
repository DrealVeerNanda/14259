package org.firstinspires.ftc.teamcode.driver;

public class Pid {
    private static double EPSILON = 0.01;

    private Pid.ErrorFunction errorFunction;
    private Pid.ResponseFunction responseFunction;
    private Pid.Coefficients coefficients;
    private Timer timer;
    private double e = 0.0;
    private double et = 0.0;
    private double deDt = 0.0;

    public static class Coefficients {
        public double kp = 0.0, ki = 0.0, kd = 0.0, deGain = 0.0, etMax = 1e+99;

        public Coefficients(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public Coefficients(double kp, double ki, double kd, double deGain, double etMax) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.deGain = deGain;
            this.etMax = etMax;
        }
    }

    public static interface ErrorFunction { public double execute(); }
    public static interface ResponseFunction { public void execute(double factor); }

    public Pid(Pid.Coefficients coefficients, Pid.ErrorFunction errorFunction, Pid.ResponseFunction responseFunction) {
        this.coefficients = coefficients;
        this.errorFunction = errorFunction;
        this.responseFunction = responseFunction;
        this.timer = new Timer();
    }

    public void update() {
        this.timer.update();

        double e = this.errorFunction.execute();
        double dt = this.timer.getDt();

        double deDt = MathUtils.lerp((e - this.e) / dt, this.deDt, this.coefficients.deGain);

        double det = e * dt;
        if (Math.abs(det) < Pid.EPSILON) this.et = 0.0;
        double et = MathUtils.clamp(this.et + det, -this.coefficients.etMax, this.coefficients.etMax);

        double factor = 0.0;
        factor += this.coefficients.kp * e;
        factor += this.coefficients.ki * et;
        factor += this.coefficients.kd * deDt;

        this.e = e;
        this.et = et;
        this.deDt = deDt;

        this.responseFunction.execute(factor);
    }
}
