package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driver.StandardDriver;

@TeleOp
public class EthanOpMode extends LinearOpMode {
    StandardDriver driver;
    double x = 0.0;
    double heading = 0.0;

    private void initDriver() {
        this.driver = new StandardDriver(hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initDriver();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) this.x = 10.0;
            if (gamepad1.y) this.x = 0.0;

            if (gamepad1.a) this.heading = Math.PI / 2.0;
            if (gamepad1.b) this.heading = 0.0;

            this.driver.setX(this.x);
            this.driver.setHeading(this.heading);

            this.driver.update();
        }
    }
}
