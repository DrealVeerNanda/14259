package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MecanumWrapper;

@Config
@TeleOp(name = "Test")
public class TestDTPID extends LinearOpMode {

    //ignore
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
/*    private double xKp = 0.06;
    private double xKi = 0.4;
    private double xKd = 0.01;
    private double yKp = 0.06; // 0.06; // 0.046;
    private double yKi = 0.4; // 0.05; // 1.17;
    private double yKd = 0.01; // 0.005; // 0.006;
    private double rKp = 0.8; //1.06
    private double rKi = 0.8;
    private double rKd = 0.06;*/
    //x pid gains
    public static double xKp = 0.1;
    public static double xKi = 0.0;
    public static double xKd = 0.00;

    //y pid gains
    public static double yKp = 0.1;
    public static double yKi = 0.0;
    public static double yKd = 0.00;

    //rotational pid gains
    public static double tKp = 0.5;
    public static double tKi = 0.000000006;
    public static double tKd = 0;
    public static double integralSumMax;
    public static double stability_thresh;
    public static double lowPassGain;

    public static double xTarget = 50;
    public static double yTarget = 0;
    public static double thetaTarget = 90;
    public static double kstatic;
    double angleNormalized = 0;
    public VoltageSensor batteryVoltageSensor;
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {

        //hardware maps
        backRightMotor = hardwareMap.get(DcMotorEx.class, "FrontRight"); //frontright motor
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "FrontLeft"); //frontLeft Motor
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "BackLeft"); //backleft motor
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "BackRight"); //backright motor

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //frontLeftMotor
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //backLeftMotor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive = new SampleMecanumDrive(hardwareMap);

        //rr localizer
        //        StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //pid ceof setting
        /*
        PIDCoefficients xCoefficients = new PIDCoefficients(xKp, xKi, xKd);
        BasicPID xControl = new BasicPID(xCoefficients);

        PIDCoefficients yCoefficients = new PIDCoefficients(yKp, yKi, yKd);
        BasicPID yControl = new BasicPID(yCoefficients);

        PIDCoefficients thetaCoefficients = new PIDCoefficients(tKp, tKi, tKd);
        BasicPID thetaControl = new BasicPID(thetaCoefficients);

        */
        while (!isStarted() && !isStopRequested()) {
        }
        while (opModeIsActive()) {
            PIDCoefficients xCoefficients = new PIDCoefficients(xKp, xKi, xKd);
            BasicPID xControl = new BasicPID(xCoefficients);

            PIDCoefficients yCoefficients = new PIDCoefficients(yKp, yKi, yKd);
            BasicPID yControl = new BasicPID(yCoefficients);

            PIDCoefficients thetaCoefficients = new PIDCoefficients(tKp, tKi, tKd);
            BasicPID thetaControl = new BasicPID(thetaCoefficients);
            //PIDCoefficientsEx coefficients = new PIDCoefficientsEx(tKp, tKi, tKd, integralSumMax, stability_thresh, lowPassGain);
// usage of the PID
            //PIDEx thetaControl = new PIDEx(coefficients);
            //updating the localizer each loop
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            angleNormalized = angleWrap(Math.toRadians(thetaTarget) - poseEstimate.getHeading());
            ;
            //Math.toDegrees(angleWrap(Math.toRadians(thetaTarget - poseEstimate.getHeading())); anti angle wrap

            double x = xControl.calculate(xTarget, poseEstimate.getX());
            double y = yControl.calculate(yTarget, poseEstimate.getY());
            double t = thetaControl.calculate(0, -angleNormalized);
            //double t = r + kstatic;

            //double x_rotated = x * Math.cos(Math.toRadians(poseEstimate.getHeading()))  - y * Math.sin(Math.toRadians(poseEstimate.getHeading())) ;
            //double y_rotated = x * Math.sin(Math.toRadians(poseEstimate.getHeading())) + y * Math.cos(Math.toRadians(poseEstimate.getHeading())) ;
            Pose2d test = new Pose2d(x,y,t);
            fieldRelative(test);
            // x, y, theta input mixing with voltage compensation
//                frontLeftMotor.setPower(((x_rotated + y_rotated + t)*12)/batteryVoltageSensor.getVoltage());
//                backLeftMotor.setPower(((x_rotated - y_rotated + t)*12)/batteryVoltageSensor.getVoltage());
//                frontRightMotor.setPower(((x_rotated - y_rotated - t)*12)/batteryVoltageSensor.getVoltage());
//                backRightMotor.setPower(((x_rotated + y_rotated - t)*12)/batteryVoltageSensor.getVoltage());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", angleNormalized *180/3.14159265358979);
            telemetry.update();
        }
    }

    public void robotRelative(Pose2d powers) {
        drive.setWeightedDrivePower(powers);
    }
    public void fieldRelative(Pose2d powers) {
        Vector2d vec = new Vector2d(powers.getX(), powers.getY());
        vec = vec.rotated(drive.getPoseEstimate().getHeading());
        powers = new Pose2d(vec.getX(), vec.getY(), powers.getHeading());
        robotRelative(powers);
    }
    public double angleWrap(double radians) {
        if (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        else if (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
