package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.managers.TimeManager;
import org.firstinspires.ftc.teamcode.wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MecanumWrapper;
import org.firstinspires.ftc.teamcode.wrappers.TwoWheelOdometryWrapper;
import org.firstinspires.ftc.teamcode.wrappers.TwoWheelOdometryDriveWrapper;
import org.firstinspires.ftc.teamcode.wrappers.PIDWrapper;
import org.firstinspires.ftc.teamcode.wrappers.ServoWrapper;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SleeveDetection;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


import org.firstinspires.ftc.teamcode.SleeveDetection;

import java.util.Random;
import com.qualcomm.robotcore.hardware.VoltageSensor;



@Autonomous
public class RightSide1Plus10 extends LinearOpMode {
    //Camera
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final double Bald_eagles_per_football_feild = 100;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    // PID Parameters
    private double xKp = 0.06;
    private double xKi = 0.4;
    private double xKd = 0.01;
    private double yKp = 0.06; // 0.06; // 0.046;
    private double yKi = 0.4; // 0.05; // 1.17;
    private double yKd = 0.01; // 0.005; // 0.006;
    private double rKp = 1.09;
    private double rKi = 0.85;
    private double rKd = 0.07;
    public double ActualVoltage;
    public VoltageSensor voltageS;
    AprilTagDetection tagOfInterest = null;
    // Control Hub Motors
    private DcMotorWrapper frontLeft;
    private DcMotorWrapper linSlide;

    // Control Hub Servos
    private ServoWrapper deposit;
    private ServoWrapper linearServo;
    private ServoWrapper clawServo1;
    private ServoWrapper frontArm;

    // Expansion Hub Motors
    private DcMotorWrapper frontRight;
    private DcMotorWrapper backRight;
    private DcMotorWrapper backLeft;
    private DcMotorWrapper turret;

    // Expansion Hub Servos
    private ServoWrapper rightArm;
    private ServoWrapper leftArm;

    // Expansion Hub Digital Sensors
    private TouchSensor leftTouchSensor;
    private TouchSensor rightTouchSensor;

    // Gamepad Wrappers

    // Meta
    private TimeManager timeManager;

    //Camera Shenanigans
    //IMU
    private BNO055IMU imu;

    //Gyro
    double getBatteryVoltage() {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        double result = voltageSensor.getVoltage();

        return result;
    }
    // Drivetrain Wrappers
    private TwoWheelOdometryDriveWrapper driveWrapper;

    // Control Parameters
    private double armLowerBound = 0.5;
    private double armUpperBound = 0.82;
    private final double clawLowerBound = 0.45;
    private final double clawUpperBound = 0.23;
    private double frontArmLowerBound = 0.04;
    private double frontArmUpperBound = 0.87;
    private int linSlideUpperBound = 1150;
    private int linSlideLowerBound = 0;
    private double linSlidePower = 1;
    private final double depositLowerBound = 0.94;
    private final double depositUpperBound = 0.19 ;
    private int turretLowerBound = 0;
    private int turretUpperBound = 2800;
    private double turretPower = 0.6;
    private double linearServoLowerBound = 0.1;
    private double linearServoUpperBound = 0.9;

    // Odometry Parameters
    private double inchesToTicks = 1901.86;
    private double degreesToTicks = 100;
    private double trackWidth = 11.25;
    private double forwardOffset = 5.35;

    // Position Parameters
    private double[] armPositions = { 0.1,0.2,0.25,0.4,0.5,0.6,0.7,0.8,0.9,1};
    private int armPosition = 7;
    private double[] clawPositions = { 0.0, 1.0 };
    private int clawPosition = 0;
    private double[] frontArmPositions = { 0.96, 0.0 };
    private int frontArmPosition = 0;
    private double[] linSlidePositions = { 0.0, 0.7, 0.79};
    private int linSlidePosition = 0;
    private double[] depositPositions = { 0.0, 0.32, 1.0 };
    private int depositPosition = 0;
    private double[] linearServoPositions = { 0.4, 0.5, 1.0 };
    private int linearServoPosition = 0;
    private double[] frontArmDifference = {0 , 0.05, 0.1};
    private double[] linSlideFactor = {1, 0.75, 0.5};
    private double lastArmPositionFactor = 1;
    private boolean armOut = false;
    private int step = 0;
    private double linSlideOffset = 0;
    private boolean linSlideHigh = true;
    private double[] speedScales = {0.4, 0.6, 0.8, 1};
    private int speedScale = 3;
    private double frontArmOffset = 0; //for full extention
    boolean buttonPressed = false;
    boolean test = true;
    int Randomization = 69;
    int rightOdometerRawValue = 0;
    private double angle;
    boolean parked = false;





    //Function
    // private static void Score(int step) {
    // private static void Score(int step) {
    //	 if(step == 0){

    private void initControlHub() {
        telemetry.addData("Status", "Initializing Control Hub");
        telemetry.update();

        // Get motors
        this.linSlide = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("LinSlide"))
                .setLowerBound(this.linSlideLowerBound)
                .setUpperBound(this.linSlideUpperBound)
                .setPower(this.linSlidePower);

        // Get servos
        this.deposit		 = new ServoWrapper()
                .setServo(hardwareMap.servo.get("Deposit"))
                .setLowerBound(this.depositLowerBound)
                .setUpperBound(this.depositUpperBound);
        this.linearServo = new ServoWrapper()
                .setServo(hardwareMap.servo.get("LinearServo"))
                .setLowerBound(this.linearServoLowerBound)
                .setUpperBound(this.linearServoUpperBound);
        this.clawServo1	= new ServoWrapper()
                .setServo(hardwareMap.servo.get("ClawServo1"))
                .setLowerBound(1.0 - this.clawLowerBound)
                .setUpperBound(1.0 - this.clawUpperBound);
        this.frontArm		= new ServoWrapper()
                .setServo(hardwareMap.servo.get("FrontArm"))
                .setLowerBound(this.frontArmLowerBound)
                .setUpperBound(this.frontArmUpperBound);

        telemetry.addData("Status", "Initialized Control Hub");
        telemetry.update();
    }
    private void initExpansionHub() {
        telemetry.addData("Status", "Initializing Expansion Hub");
        telemetry.update();

        // Get motors
        // this.turret		 = new DcMotorWrapper()
        // 	.setDcMotor(hardwareMap.dcMotor.get("Turret"), true)
        // 	.setLowerBound(this.turretLowerBound)
        // 	.setUpperBound(this.turretUpperBound)
        // 	.setPower(this.turretPower);
        this.turret		 = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("Turret"), true)
                .setLowerBound(this.turretLowerBound)
                .setUpperBound(this.turretUpperBound)
                .setPower(this.turretPower);
        ;

        // Get servos
        this.leftArm	= new ServoWrapper()
                .setServo(hardwareMap.servo.get("LeftArm"))
                .setLowerBound(1.0 - armLowerBound)
                .setUpperBound(1.0 - armUpperBound);
        this.rightArm = new ServoWrapper()
                .setServo(hardwareMap.servo.get("RightArm"))
                .setLowerBound(armLowerBound)
                .setUpperBound(armUpperBound);

        // Get digital sensors
        this.leftTouchSensor	= hardwareMap.touchSensor.get("LeftTouchSensor");
        this.rightTouchSensor = hardwareMap.touchSensor.get("RightTouchSensor");

        telemetry.addData("Status", "Initialized Expansion Hub");
        telemetry.update();
    }
    private void initPIDs() {
        telemetry.addData("Status", "Initializing PIDs");
        telemetry.update();

        telemetry.addData("Status", "Initialized PIDs");
        telemetry.update();
    }
    private void initSensors() {
        telemetry.addData("Status", "Initializing Sensors");
        telemetry.update();

        // Initialize IMU
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //parameters.mode = BNO055IMU.SensorMode.GYRONLY;
        this.imu.initialize(parameters);
        this.imu.write8(BNO055IMU.Register.OPR_MODE, 0b00000011);
        telemetry.addData("Status", "Initialized Sensors");
        telemetry.update();
    }
    private void initDrivetrain() {
        telemetry.addData("Status", "Initializing Drivetrain");
        telemetry.update();

        // Create Mecanum wrapper
        MecanumWrapper mecanumWrapper = new MecanumWrapper()
                .setFrontLeft(hardwareMap.dcMotor.get("FrontLeft"))
                .setFrontRight(hardwareMap.dcMotor.get("FrontRight"))
                .setBackLeft(hardwareMap.dcMotor.get("BackLeft"))
                .setBackRight(hardwareMap.dcMotor.get("BackRight"));

        // Reverse FrontLeft and BackLeft
        mecanumWrapper
                .getFrontLeft()
                .setDirection(DcMotor.Direction.REVERSE);
        mecanumWrapper
                .getBackLeft()
                .setDirection(DcMotor.Direction.REVERSE);

        // Create Odometry wrapper
        TwoWheelOdometryWrapper odometryWrapper = new TwoWheelOdometryWrapper()
                .setParaEncoder(hardwareMap.dcMotor.get("BackLeft"))
                .setPerpEncoder(hardwareMap.dcMotor.get("FrontRight"))
                .setParaOffset(this.trackWidth / 2.0)
                .setPerpOffset(this.forwardOffset)
                .setImu(this.imu);

        // Create Drive Wrapper
        this.driveWrapper = new TwoWheelOdometryDriveWrapper()
                .setMecanumWrapper(mecanumWrapper)
                .setOdometryWrapper(odometryWrapper);

        // Set PID values
        this.driveWrapper
                .getXPIDWrapper()
                .setKp(this.xKp)
                .setKi(this.xKi)
                .setKd(this.xKd)
                .setVoltage(getBatteryVoltage());
        this.driveWrapper
                .getYPIDWrapper()
                .setKp(this.yKp)
                .setKi(this.yKi)
                .setKd(this.yKd)
                .setVoltage(getBatteryVoltage());
        this.driveWrapper
                .getRPIDWrapper()
                .setKp(this.rKp)
                .setKi(this.rKi)
                .setKd(this.rKd)
                .setVoltage(getBatteryVoltage());


        telemetry.addData("Status", "Initialized Drivetrain");
        telemetry.update();
    }
    private void initMeta() {
        telemetry.addData("Status", "Initializing Meta");
        telemetry.update();

        // Create timeManager
        this.timeManager = new TimeManager()
                .setOpMode(this);

        // Set timeManager
        DcMotorWrapper.setTimeManager(this.timeManager);
        ServoWrapper.setTimeManager(this.timeManager);
        PIDWrapper.setTimeManager(this.timeManager);

        telemetry.addData("Status", "Initialized Meta");
        telemetry.update();
    }
    private void initProcesses() {
        telemetry.addData("Status", "Initializing Processes");
        telemetry.update();

        telemetry.addData("Status", "Initialized Processes");
        telemetry.update();
    }
    private void initPositions() {
        telemetry.addData("Status", "Initializing Positions");
        telemetry.update();

        // Initialize Control Hub motor positions
        //this.linSlide.setPosition(this.linSlidePositions[this.linSlidePosition]);

        // Initialize Control Hub servo positions
        this.deposit.setPosition(this.depositPositions[this.depositPosition]);
        this.clawServo1.setPosition(this.clawPositions[this.clawPosition]);
        this.frontArm.setPosition(this.frontArmPositions[this.frontArmPosition]);

        // Initialize Expansion Hub servo positions
        this.leftArm.setPosition(0);
        this.rightArm.setPosition(0);

        telemetry.addData("Status", "Initialized Positions");
        telemetry.update();
    }
    private void initAll() {
        //telemetry.addData("ROTATION: ", sleeve.getPosition());
        telemetry.addData("Status", "Initializing all");
        telemetry.update();

        // Initialize everything
        this.initControlHub();
        this.initExpansionHub();
        this.initPIDs();
        this.initSensors();
        this.initDrivetrain();
        this.initMeta();
        this.initProcesses();
        this.initPositions();

        frontArm.setPosition(frontArmPositions[frontArmPosition]);

        telemetry.addData("Status", "Initialized all");
        telemetry.update();
    }
    private void updateAll() {
        // Control Hub motors
        this.linSlide.update();

        // Control Hub servos
        this.deposit.update();
        this.linearServo.update();
        this.clawServo1.update();
        this.frontArm.update();

        // Expansion Hub motors
        this.turret.update();

        // Expansion Hub servos
        this.rightArm.update();
        this.leftArm.update();

        // Gamepads update

        // Drivetrain update
        this.angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if(!parked) this.driveWrapper.update();
        else this.driveWrapper.getMecanumWrapper().update();

        this.displayStats();

        //IMU Angle

        // Meta update
        this.timeManager.update();

    }
    private void interact() {
        this.turret.getDcMotor().setPower(0.7 * (gamepad2.left_stick_x));
        this.turret.getDcMotor().setPower(0.7 * (gamepad1.right_trigger - gamepad1.left_trigger));
    }
    private void displayStats() {
        telemetry.addData("Speed Scale ", speedScales[speedScale]);
        telemetry.addData("Arm Position ", armPositions[armPosition]);
//    telemetry.addData("Linear Slide Current Offset ", linSlideOffset);
        telemetry.addData("LinSlide raw encoder position", this.linSlide.getDcMotor().getCurrentPosition());
        telemetry.addData("Turret raw encoder position", this.turret.getDcMotor().getCurrentPosition());

        telemetry.addData("Randomization: ", Randomization);
        telemetry.addData("Raw Value ", rightOdometerRawValue);
        telemetry.addData("isBusy ", driveWrapper.isBusy());
//    telemetry.addData("Temperature of Robot(real)", imu.getTemperature());
//    telemetry.addData("Angular V", imu.getAngularVelocity());

        telemetry.addData("Angle", angle);

        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        //voltageS = hardwareMap.get(VoltageSensor.class, "battery");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        this.displayStats();
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                    telemetry.addLine("LEFT");
                    Randomization = 1;
                } else if (tagOfInterest.id == MIDDLE) {
                    telemetry.addLine("MIDDLE");
                    Randomization = 2;
                } else {
                    telemetry.addLine("RIGHT");
                    Randomization = 3;
                }
                telemetry.update();

            }
        }

        while (opModeIsActive()) {
            this.linearServo.setPosition(0.25);
            this.turret.setPosition(0.8);
            this.driveWrapper.setPose(0, 50, 0);
            for(int i = 0; i < 34; i++) this.updateAll();
            frontArm.setPosition(0.8);
            this.driveWrapper.setPose(0, 50, Math.PI / 2.0);
            for(int i = 0; i < 7; i++) this.updateAll();
            this.driveWrapper.setPose(-10, 50, Math.PI / 2.0);
            for(int i = 0; i < 12; i++) this.updateAll();
            parked = true;
            sleep(4000);

//            updateAll();
//            this.displayStats();
//            for (int i = 5; i > 0; i--) {
//                intakeOut(i);
//                sleep(150);
//                armPosition = 7;
//                moveArm(7);
//                resetLinSlide();
//                sleep(150);
//                intakeBack();
//                sleep(750);
//                linSlideUp(i);
//                preIntakeMode(i);
//                sleep(400);
//                dump();
//                sleep(300);
//            }
//            dump();
//            updateAll();
//            greatReset();
//            turret.setPosition(0.1);
//            updateAll();
//            sleep(500);
//            this.displayStats();
            parked = false;
            this.driveWrapper.setPose(72, 50, Math.PI / 2.0);
            for(int i = 0; i < 55; i++) this.updateAll();
            this.driveWrapper.setPose(72, 50, 0);
            for(int i = 0; i < 8; i++) this.updateAll();
            this.driveWrapper.setPose(72, 50, -Math.PI / 1.8);
            for(int i = 0; i < 16; i++) this.updateAll();
            sleep(100000);
            park();
            parked = true;
            sleep(50000);

        }
    }
    private void park(){
        parked = false;
        if (Randomization == 1) {
            this.driveWrapper.setPose(0 + 24, 50, Math.PI / 2);
            for(int i = 0; i < 40; i++) this.updateAll();
            parked = true;
        } else if (Randomization == 2) {
            parked = true;
        } else {
            this.driveWrapper.setPose(0 - 24, 50, Math.PI / 2);
            for(int i = 0; i < 40; i++) this.updateAll();
            parked = true;
        }
    }
    private void moveClaw(){
        this.clawServo1.setPosition(this.clawPositions[clawPosition]);
        updateAll();
    }
    private void moveLinSlide(){
        this.linSlide.setPosition(this.linSlidePositions[linSlidePosition]*linSlideFactor[linearServoPosition]+ linSlidePositions[linSlidePosition] * linSlideOffset);
        updateAll();
    }
    private void moveArm(int position){
        if(position < 0 ||position > 9){
            this.leftArm.setPosition(0);
            this.rightArm.setPosition(0);
            armOut = false;
        }else{
            this.leftArm.setPosition(this.armPositions[position]);
            this.rightArm.setPosition(this.armPositions[position]);
            armOut = true;
        }
        updateAll();
    }
    private void moveLinearServo(){
        this.linearServo.setPosition(this.linearServoPositions[linearServoPosition]);
        updateAll();
    }
    private void moveDeposit(){
        this.deposit.setPosition(this.depositPositions[depositPosition]);
        updateAll();
    }
    private void moveFrontArm(){
        if(frontArmPositions[frontArmPosition] - frontArmDifference[linearServoPosition] < 0)frontArm.setPosition(0);
        else this.frontArm.setPosition(frontArmPositions[frontArmPosition] - frontArmDifference[linearServoPosition]);
        updateAll();
    }
    private void intakeOut(int i){
        frontArmPosition = 1;
        intakeArm(i);
        moveArm(armPosition);
        clawPosition = 0;
        moveClaw();
    }
    private void intakeBack(){
        clawPosition = 1;
        moveClaw();
        //All Sleep function will be replaced by time mangers.
        sleep(250);
        frontArmPosition = 0;
        sleep(350);
        moveFrontArm();
        moveArm(-1);
    }
    private void linSlideUp(int i){
        clawPosition = 0;
        moveClaw();
        sleep(700);
        if(linSlideHigh) linSlidePosition = 2;
        else linSlidePosition = 1;
        moveLinSlide();
        sleep(100);
        depositPosition = 1;
        moveDeposit();
        if(linSlideHigh) preIntakeMode(i);
    }
    private void resetLinSlide(){
        if(depositPosition != 0){
            depositPosition = 0;
            moveDeposit();
            sleep(200);
        }
        linSlidePosition = 0;
        moveLinSlide();
        //Add turret center code here later!
    }
    private void preIntakeMode(int i){
        frontArmPosition = 1;
        intakeArm(i);
        moveArm(armPosition-1);
    }
    private void dump(){
        depositPosition = 2;
        moveDeposit();
    }
    private void greatReset(){
        resetLinSlide();
        intakeBack();
        step = 0;
    }
    private void intakeArm(int i){
        if(i == 5) frontArm.setPosition(frontArmPositions[frontArmPosition] + 0.13);
        else if(i == 4) frontArm.setPosition(frontArmPositions[frontArmPosition] + 0.11);
        else if(i == 3) frontArm.setPosition(frontArmPositions[frontArmPosition] + 0.07);
        else if(i==2) frontArm.setPosition(frontArmPositions[frontArmPosition] + 0.01 * i);
        else if(i==1) frontArm.setPosition(frontArmPositions[frontArmPosition] + 0.01 * i);
        else frontArm.setPosition(0);

    }
}
