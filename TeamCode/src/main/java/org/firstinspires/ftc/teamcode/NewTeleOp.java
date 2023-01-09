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
import org.firstinspires.ftc.teamcode.managers.TimeManager;
import org.firstinspires.ftc.teamcode.wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MecanumWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OdometryWrapper;
import org.firstinspires.ftc.teamcode.wrappers.PIDWrapper;
import org.firstinspires.ftc.teamcode.wrappers.ServoWrapper;
@Disabled
@TeleOp
public class NewTeleOp extends LinearOpMode {
    // Control Hub Motors
    private DcMotorWrapper linSlideLeft;
    private DcMotorWrapper linSlideRight;

    // Control Hub Servos
    private ServoWrapper turret;
    private ServoWrapper clawServo1;

    // Expansion Hub Motors
    private DcMotorWrapper frontRight;
    private DcMotorWrapper backRight;
    private DcMotorWrapper backLeft;

    // Expansion Hub Servos
    private ServoWrapper odometer;
    private ServoWrapper rightArm;
    private ServoWrapper leftArm;

    // Expansion Hub Digital Sensors
    // Gamepad Wrappers
    private GamepadWrapper gamepad1Wrapper;
    private GamepadWrapper gamepad2Wrapper;

    // Meta
    private TimeManager timeManager;

    //IMU
    // Drivetrain Wrappers
    private MecanumWrapper mecanumWrapper;

    // Control Parameters
    private final double armLowerBound = 0;
    private final double armUpperBound = 0.65;
    private final double clawLowerBound = 0.1;
    private final double clawUpperBound = 0.5;
    private final int linSlideLowerBound = 0;
    private final int linSlideUpperBound = 2500;
    private final double linSlidePower = 1;
    private final double turretUpperBound = 0.65;
    private final double turretLowerBound = 0;
    private final double odometerUpperBound = 0.5;
    private final double odometerLowerBound = 0.2;


    // Position Parameters
    private final double[] armPositions = {0.0 , 0.8};
    private int armPosition = 0;
    private final double[] odometerPositions = {0,1};
    private int odometerPosition = 1;
    private final double[] clawPositions = { 0.0, 0.5};
    private int clawPosition = 0;
    private final double[] linSlidePositions = { 0.0, 0.45, 0.6, 0.88,};
    private int linSlidePosition = 0;
    private final double[] turretPositions = { 0.0, 1.0};
    private int turretPosition = 0;
    private boolean linSlideUp = false;
    private boolean clawOut = false;
    private double linSlideOffset = 0;
    private final double[] speedScales = {0.4, 0.6, 0.8, 1};
    private int speedScale = 2;
    private double deltaR = 1;

    //Function
    // private static void Score(int step) {
    // private static void Score(int step) {
    //	 if(step == 0){

    private void initControlHub() {
        telemetry.addData("Status", "Initializing Control Hub");
        telemetry.update();

        // Get motors
        this.linSlideLeft = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("leftSlide"))
                .setLowerBound(this.linSlideLowerBound)
                .setUpperBound(-this.linSlideUpperBound)
                .setPower(-this.linSlidePower);
        this.linSlideRight = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("rightSlide"))
                .setLowerBound(this.linSlideLowerBound)
                .setUpperBound(this.linSlideUpperBound)
                .setPower(this.linSlidePower);

        // Get servos
        this.clawServo1	= new ServoWrapper()
                .setServo(hardwareMap.servo.get("openWide"))
                .setLowerBound(this.clawLowerBound)
                .setUpperBound(this.clawUpperBound);
        this.odometer	= new ServoWrapper()
                .setServo(hardwareMap.servo.get("retraction"))
                .setLowerBound(this.odometerLowerBound)
                .setUpperBound(this.odometerUpperBound);
        this.turret		= new ServoWrapper()
                .setServo(hardwareMap.servo.get("servoTurret"))
                .setLowerBound(this.turretLowerBound)
                .setUpperBound(this.turretUpperBound);

        telemetry.addData("Status", "Initialized Control Hub");
        telemetry.update();
    }
    private void initExpansionHub() {
        telemetry.addData("Status", "Initializing Expansion Hub");
        telemetry.update();

        // Get motors
        // Get servos
        this.leftArm	= new ServoWrapper()
                .setServo(hardwareMap.servo.get("leftv4b"))
                .setLowerBound(armLowerBound)
                .setUpperBound(armUpperBound);
        this.rightArm = new ServoWrapper()
                .setServo(hardwareMap.servo.get("rightv4b"))
                .setLowerBound(armLowerBound)
                .setUpperBound(armUpperBound);

        // Get digital sensors
        telemetry.addData("Status", "Initialized Expansion Hub");
        telemetry.update();
    }
    private void initGamepads() {
        telemetry.addData("Status", "Initializing Gamepads");
        telemetry.update();

        // Get gamepad1
        this.gamepad1Wrapper = new GamepadWrapper()
                .setGamepad(gamepad1);

        this.gamepad2Wrapper = new GamepadWrapper()
                .setGamepad(gamepad2);
        //dPad
        //!!!!
        //

        //

        this.gamepad2Wrapper.subscribeLPressedEvent(() -> {
            return false;
        });
        this.gamepad2Wrapper.subscribeRPressedEvent(() -> {
            this.speedScale = (this.speedScale + 1) % this.speedScales.length;
            return false;
        });
        this.gamepad2Wrapper.subscribeDPressedEvent(() -> {
            return false;
        });
        this.gamepad2Wrapper.subscribeUPressedEvent(() -> {
            return false;
        });
        this.gamepad1Wrapper.subscribeLPressedEvent(() -> {
            linSlideOffset = 0;
            moveLinSlide();
            return false;
        });
        this.gamepad1Wrapper.subscribeRPressedEvent(() -> {
            return false;
        });
        this.gamepad1Wrapper.subscribeDPressedEvent(() -> {
            if(linSlideOffset > 0) linSlideOffset -= 0.04;
            moveLinSlide();
            return false;
        });
        this.gamepad1Wrapper.subscribeUPressedEvent(() -> {
            if(linSlideOffset < 0.2) linSlideOffset += 0.04;
            moveLinSlide();
            return false;
        });
        //Bumpers





        this.gamepad2Wrapper.subscribeRbPressedEvent(() -> {
            this.clawPosition = (this.clawPosition + 1) % this.clawPositions.length;
            moveClaw();
            return false;
        });
        this.gamepad2Wrapper.subscribeLbPressedEvent(() -> {
            return false;
        });
        this.gamepad1Wrapper.subscribeRbPressedEvent(() -> {
            this.clawPosition = 0;
            moveClaw();
            return false;
        });
        this.gamepad1Wrapper.subscribeLbPressedEvent(() -> {
            this.clawPosition = 1;
            moveClaw();
//            this.odometerPosition = (this.odometerPosition + 1) % this.odometerPositions.length;
//            moveOdometer();
            return false;
        });

        //letters


        this.gamepad1Wrapper.subscribeYPressedEvent(() -> {
            clawOut(3);
            return false;
        });
        this.gamepad1Wrapper.subscribeAPressedEvent(() -> {
            this.linSlidePosition = 1;
            moveLinSlide();
            return false;
        });
        this.gamepad1Wrapper.subscribeBPressedEvent(() -> {
            clawOut(2);
            return false;
        });
        this.gamepad1Wrapper.subscribeXPressedEvent(() -> {
            greatReset();
            return false;
        });
        this.gamepad2Wrapper.subscribeYPressedEvent(() -> {
            return false;
        });
        this.gamepad2Wrapper.subscribeAPressedEvent(() -> {
            this.armPosition = (this.armPosition + 1) % this.armPositions.length;
            move4bar();
            return false;
        });
        this.gamepad1Wrapper.subscribeBPressedEvent(() -> {
            this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
            moveLinSlide();
            return false;
        });
        this.gamepad1Wrapper.subscribeXPressedEvent(() -> {
            return false;
        });
        telemetry.addData("Status", "Initialized Gamepads");
        telemetry.update();
    }


    private void initDrivetrain() {
        telemetry.addData("Status", "Initializing Drivetrain");
        telemetry.update();

        // Create Mecanum wrapper
        this.mecanumWrapper = new MecanumWrapper()
                .setFrontLeft(hardwareMap.dcMotor.get("leftFront"))
                .setFrontRight(hardwareMap.dcMotor.get("rightFront"))
                .setBackLeft(hardwareMap.dcMotor.get("leftRear"))
                .setBackRight(hardwareMap.dcMotor.get("rightRear"));

        // Reverse FrontLeft and BackLeft
        this.mecanumWrapper
                .getFrontRight()
                .setDirection(DcMotor.Direction.REVERSE);
        this.mecanumWrapper
                .getBackRight()
                .setDirection(DcMotor.Direction.REVERSE);

        // Create Odometry wrapper
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
        this.linSlideLeft.setPosition(this.linSlidePositions[this.linSlidePosition]);
        this.linSlideLeft.setPosition(this.linSlidePositions[this.linSlidePosition]);
        this.clawServo1.setPosition(this.clawPositions[this.clawPosition]);
        this.turret.setPosition(this.turretPositions[this.turretPosition]);

        // Initialize Expansion Hub servo positions
        this.leftArm.setPosition(this.armPositions[armPosition]);
        this.rightArm.setPosition(this.armPositions[armPosition]);
        this.odometer.setPosition(this.odometerPositions[this.odometerPosition]);

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
        this.initDrivetrain();
        this.initMeta();
        this.initProcesses();
        this.initPositions();

        telemetry.addData("Status", "Initialized all");
        telemetry.update();
    }

    private void updateAll() {
        // Control Hub motors
        this.linSlideLeft.update();

        // Control Hub servos
        this.linSlideRight.update();
        this.clawServo1.update();
        this.leftArm.update();
        this.rightArm.update();
        // Expansion Hub motors
        this.turret.update();

        // Gamepads update
        this.gamepad1Wrapper.update();
        this.gamepad2Wrapper.update();

        // Drivetrain update
        this.mecanumWrapper.update();

        // Meta update
        this.timeManager.update();
    }

    private void interact() {
        this.mecanumWrapper.setPowerX(speedScales[speedScale] * -gamepad1.left_stick_x);
        this.mecanumWrapper.setPowerY(-speedScales[speedScale] * gamepad1.left_stick_y * 1.1 * deltaR);
        this.mecanumWrapper.setPowerR(speedScales[speedScale] * -gamepad1.right_stick_x * 0.8 * deltaR);
    }

    private void displayStats() {
//        telemetry.addData("left", this.odometryWrapper.getLeftEncoderPosition());
//        telemetry.addData("right", this.odometryWrapper.getRightEncoderPosition());
//        telemetry.addData("front", this.odometryWrapper.getFrontEncoderPosition());
//        telemetry.addData("X", this.odometryWrapper.getX() / this.inchesToTicks);
//        telemetry.addData("Y", this.odometryWrapper.getY() / this.inchesToTicks);
//        telemetry.addData("R", this.odometryWrapper.getR() / this.degreesToTicks);
        telemetry.addData("Speed Scale ", speedScales[speedScale]);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            this.displayStats();
            new Thread(()-> {this.interact();
                updateAll();}).start();
            this.updateAll();
        }
    }
    private void moveClaw(){
        this.clawServo1.setPosition(this.clawPositions[clawPosition]);
        updateAll();
    }
    private void move4bar(){

            this.leftArm.setPosition(this.armPositions[armPosition]);
            this.rightArm.setPosition(this.armPositions[armPosition]);
            updateAll();
    }
    private void moveTurret(){
        this.turret.setPosition(this.turretPositions[turretPosition]);
        updateAll();
    }
    private void moveLinSlide(){
        this.linSlideLeft.setPosition(this.linSlidePositions[linSlidePosition] + linSlideOffset);
        this.linSlideRight.setPosition(this.linSlidePositions[linSlidePosition] + linSlideOffset);
        updateAll();
    }
    private void moveOdometer(){
        this.odometer.setPosition(this.odometerPositions[this.odometerPosition]);
        updateAll();
    }
    private void greatReset(){
        linSlideOffset = 0;
        deltaR = 1;
        clawPosition = 1;//1 is closed
        moveClaw();
            turretPosition = 0;
            moveTurret();
            if(clawOut) {
                sleep(500);
            }
            armPosition = 0;
            move4bar();
            if(clawOut) {
                sleep(500);
            }
            linSlidePosition = 0;
            moveLinSlide();
            clawOut = false;
    }
    private void clawOut(int level){
        linSlideOffset = 0;
        deltaR = 0.6;
        clawPosition = 1;
        linSlidePosition = level;
        moveLinSlide();
        moveClaw();
        if(!clawOut) {
            clawOut = true;
            updateAll();
            sleep(500);
            turretPosition = 1;
            moveTurret();
            updateAll();
            sleep(600);
            armPosition = 1;
            move4bar();
        }
    }

}
