
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.managers.EventManager;
import org.firstinspires.ftc.teamcode.managers.TimeManager;
import org.firstinspires.ftc.teamcode.wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MecanumWrapper;
import org.firstinspires.ftc.teamcode.wrappers.PIDWrapper;
import org.firstinspires.ftc.teamcode.wrappers.ServoWrapper;
import org.firstinspires.ftc.teamcode.wrappers.TwoWheelOdometryDriveWrapper;
import org.firstinspires.ftc.teamcode.wrappers.TwoWheelOdometryWrapper;
@TeleOp
public class OdometryTeleOp extends LinearOpMode {
    // Control Hub DcMotors
    private DcMotorWrapper frontLeft;
    private DcMotorWrapper linSlide;

    // Control Hub Servos
    private ServoWrapper deposit;
    private ServoWrapper linearServo;
    private ServoWrapper clawServo1;
    private ServoWrapper frontArm;

    // Expansion Hub DcMotors
    private DcMotorWrapper frontRight;
    private DcMotorWrapper backRight;
    private DcMotorWrapper backLeft;
    private DcMotorWrapper turret;

    // Expansion Hub Servos
    private ServoWrapper rightArm;
    private ServoWrapper leftArm;

    // Gamepad Wrappers
    private GamepadWrapper gamepadWrapper;

    // Meta
    private TimeManager timeManager;

    // Drivetrain Wrappers
    private TwoWheelOdometryDriveWrapper driveWrapper;

    // Odometry Parameters
    private double trackWidth = 12.0;
    private double forwardOffset = 5.1875;

    // Sensors
    private BNO055IMU imu;

    // Control Bounds
    private double armLowerBound = 0.5;
    private double armUpperBound = 0.82;
    private double clawLowerBound = 0.55;
    private double clawUpperBound = 0.85;
    private double frontArmLowerBound = 0.04;
    private double frontArmUpperBound = 0.87;
    private int linSlideLowerBound = 0;
    private int linSlideUpperBound = 1000;
    private double linSlidePower = 1;
    private double depositLowerBound = 0.53;
    private double depositUpperBound = 0.98;
    private int turretLowerBound = 0;
    private int turretUpperBound = 2060;
    private double turretPower = 0.8;
    private double linearServoLowerBound = 0.1;
    private double linearServoUpperBound = 0.9;

    // PID Parameters
    private double xKp = 0.06;
    private double xKi = 0.5;
    private double xKd = 0.005;
    private double yKp = 0.06; // 0.06; // 0.046;
    private double yKi = 0.5; // 0.05; // 1.17;
    private double yKd = 0.005; // 0.005; // 0.006;
    private double rKp = 1.09;
    private double rKi = 0.85;
    private double rKd = 0.04;

    // Toggle Positions
    private double[] yPositions = { 0.0, 50.0 };
    private int yPosition = 0;
    private double[] rPositions = { 0.0, Math.PI / 2.0 };
    private int rPosition = 0;

    private void initControlHub() {
        telemetry.addData("Status", "Initializing Control Hub");
        telemetry.update();
        
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

        // Get servos
        this.leftArm	= new ServoWrapper()
                .setServo(hardwareMap.servo.get("LeftArm"))
                .setLowerBound(1.0 - armLowerBound)
                .setUpperBound(1.0 - armUpperBound);
        this.rightArm = new ServoWrapper()
                .setServo(hardwareMap.servo.get("RightArm"))
                .setLowerBound(armLowerBound)
                .setUpperBound(armUpperBound);

        // Initialize servo positions
        this.leftArm.setPosition(0.0);
        this.rightArm.setPosition(0.0);

        telemetry.addData("Status", "Initialized Expansion Hub");
        telemetry.update();
    }

    private void initSensors() {
        telemetry.addData("Status", "Initializing Sensors");
        telemetry.update();

        // Initialize IMU
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);

        telemetry.addData("Status", "Initialized Sensors");
        telemetry.update();
    }

    private void initGamepads() {
        telemetry.addData("Status", "Initializing Gamepads");
        telemetry.update();

        this.gamepadWrapper = new GamepadWrapper()
                .setGamepad(gamepad1);

        telemetry.addData("Status", "Initialized Gamepads");
        telemetry.update();
    }

    private void initPIDs() {
        telemetry.addData("Status", "Initializing PIDs");
        telemetry.update();

        telemetry.addData("Status", "Initialized PIDs");
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

        this.gamepadWrapper.subscribeYPressedEvent(() -> {
            this.rPosition = (this.rPosition + 1) % this.rPositions.length;
            this.driveWrapper.setPose(0.0, this.yPositions[this.yPosition], this.rPositions[this.rPosition]);
            return false;
        });

        this.gamepadWrapper.subscribeBPressedEvent(() -> {
            this.yPosition = (this.yPosition + 1) % this.yPositions.length;
            this.driveWrapper.setPose(0.0, this.yPositions[this.yPosition], this.rPositions[this.rPosition]);
            return false;
        });

        // Async autonEntry = new Async();
        // Async gotoPose1 = new Async(this.driveWrapper.gotoPose(0.0, 51.0, 0.0));
        // Async gotoPose2 = new Async(this.driveWrapper.gotoPose(0.0, 51.0, Math.PI / 2.0));
        // Async leftArmOut = new Async(this.leftArm.gotoPosition(1.0, 300));
        // Async rightArmOut = new Async(this.rightArm.gotoPosition(0.0, 300));
        // Async frontArmDown = new Async(this.frontArm.gotoPosition(0.0, 300));
        // Async leftClawClose = new Async(this.leftClaw.gotoPosition(0.0, 300));
        // Async rightClawClose = new Async(this.rightClaw.gotoPosition(0.0, 300));
        // Async frontArmUp = new Async(this.frontArm.gotoPosition(0.0, 300));
        // Async leftArmIn = new Async(this.leftArm.gotoPosition(0.0, 300));
        // Async rightArmIn = new Async(this.rightArm.gotoPosition(1.0, 300));
        // Async leftClawOpen = new Async(this.leftClaw.gotoPosition(0.6, 300));
        // Async rightClawOpen = new Async(this.rightClaw.gotoPosition(0.6, 300));
        // Async linSlideUp = new Async(this.linSlide.gotoPosition(0.8, 300));
        // Async depositDown = new Async(this.deposit.gotoPosition(0.8, 300));
        // Async depositUp = new Async(this.deposit.gotoPosition(0.0, 300));
        // Async linSlideDown = new Async(this.linSlide.gotoPosition(0.0, 300));
        // Async.addLink(autonEntry, gotoPose1);
        // Async.addLink(gotoPose1, gotoPose2);
        // Async.addLink(gotoPose2, leftArmOut);
        // Async.addLink(gotoPose2, rightArmOut);
        // Async.addLink(leftArmOut, frontArmDown);
        // Async.addLink(rightArmOut, frontArmDown);
        // Async.addLink(frontArmDown, leftClawClose);
        // Async.addLink(frontArmDown, rightClawClose);
        // Async.addLink(leftClawClose, frontArmUp);
        // Async.addLink(rightClawClose, frontArmUp);
        // Async.addLink(frontArmUp, leftArmIn);
        // Async.addLink(frontArmUp, rightArmIn);
        // Async.addLink(leftArmIn, leftClawOpen);
        // Async.addLink(rightArmIn, leftClawOpen);
        // Async.addLink(leftArmIn, rightClawOpen);
        // Async.addLink(rightArmIn, rightClawOpen);
        // Async.addLink(leftClawOpen, linSlideUp);
        // Async.addLink(rightClawOpen, linSlideUp);
        // Async.addLink(linSlideUp, depositDown);
        // Async.addLink(depositDown, depositUp);
        // Async.addLink(depositUp, linSlideDown);
        // Async.addLink(linSlideDown, autonEntry);

        telemetry.addData("Status", "Initialized Processes");
        telemetry.update();
    }

    private void initPositions() {
        telemetry.addData("Status", "Initializing Positions");
        telemetry.update();

        this.frontArm.setPosition(0.8);

        telemetry.addData("Status", "Initialized Positions");
        telemetry.update();
    }

    private void initAll() {
        telemetry.addData("Status", "Initializing all");
        telemetry.update();

        // Initialize everything
        this.initControlHub();
        this.initExpansionHub();
        this.initGamepads();
        this.initPIDs();
        this.initSensors();
        this.initDrivetrain();
        this.initMeta();
        this.initProcesses();
        this.initPositions();

        telemetry.addData("Status", "Initialized all");
        telemetry.update();
    }

    private void updateAll() {
        // Gamepad update
        this.gamepadWrapper.update();

        // Drivetrain update
        this.driveWrapper.update();

        // Meta update
        this.timeManager.update();
    }

    private void interact() {
        this.yKp += gamepad1.left_stick_y * 0.01;
        this.yKi += gamepad1.right_stick_y * 0.01;
        this.yKd += (gamepad1.left_trigger - gamepad1.right_trigger) * 0.01;

        this.yKp = Math.max(this.yKp, 0.0);
        this.yKi = Math.max(this.yKi, 0.0);
        this.yKd = Math.max(this.yKd, 0.0);

        // this.xKp = this.yKp;
        // this.xKi = this.yKi;
        // this.xKd = this.yKd;

        this.rKp += gamepad2.left_stick_y * 0.01;
        this.rKi += gamepad2.right_stick_y * 0.01;
        this.rKd += (gamepad2.left_trigger - gamepad2.right_trigger) * 0.01;

        this.rKp = Math.max(this.rKp, 0.0);
        this.rKi = Math.max(this.rKi, 0.0);
        this.rKd = Math.max(this.rKd, 0.0);

        this.driveWrapper
                .getXPIDWrapper()
                .setKp(this.xKp)
                .setKi(this.xKi)
                .setKd(this.xKd);

        this.driveWrapper
                .getYPIDWrapper()
                .setKp(this.yKp)
                .setKi(this.yKi)
                .setKd(this.yKd);

        this.driveWrapper
                .getRPIDWrapper()
                .setKp(this.rKp)
                .setKi(this.rKi)
                .setKd(this.rKd);
    }

    private void displayStats() {
        telemetry.addData("X", this.driveWrapper.getOdometryWrapper().getX());
        telemetry.addData("Y", this.driveWrapper.getOdometryWrapper().getY());
        telemetry.addData("R (Radians)", this.driveWrapper.getOdometryWrapper().getR());
        telemetry.addData("R (Degrees)", this.driveWrapper.getOdometryWrapper().getR() / Math.PI * 180.0);
        telemetry.addData("Para Raw", this.driveWrapper.getOdometryWrapper().getParaEncoder().getCurrentPosition());
        telemetry.addData("Perp Raw", this.driveWrapper.getOdometryWrapper().getPerpEncoder().getCurrentPosition());
        telemetry.addData("isBusy", this.driveWrapper.isBusy());
        telemetry.addData("xKp", this.xKp);
        telemetry.addData("xKi", this.xKi);
        telemetry.addData("xKd", this.xKd);
        telemetry.addData("yKp", this.yKp);
        telemetry.addData("yKi", this.yKi);
        telemetry.addData("yKd", this.yKd);
        telemetry.addData("rKp", this.rKp);
        telemetry.addData("rKi", this.rKi);
        telemetry.addData("rKd", this.rKd);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            this.displayStats();
            this.interact();
            this.updateAll();
        }
    }
}
