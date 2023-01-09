package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.wrappers.TwoWheelOdometryDriveWrapper;

public class LeftSidePIDAutonDriveThread implements Runnable {
    private Thread thread;
    private String threadName = "Drive";
    private TwoWheelOdometryDriveWrapper driveWrapper;
    private int parking;
    private ElapsedTime elapsedTime;

    public LeftSidePIDAutonDriveThread(TwoWheelOdometryDriveWrapper driveWrapper, int parking) {
        this.driveWrapper = driveWrapper;
        this.parking = parking;
        this.elapsedTime = new ElapsedTime();
    }

    public void run() {
        this.driveWrapper.setPose(0, 51, 0);
        while (this.driveWrapper.isBusy()) this.driveWrapper.update();
        this.driveWrapper.setPose(0, 51, -Math.PI / 2.0);
        while (this.driveWrapper.isBusy()) this.driveWrapper.update();
        while (this.elapsedTime.seconds() < 28) this.driveWrapper.update();
        if (this.parking == 1) {
            this.driveWrapper.setPose(0 + 24, 51, Math.PI / 2);
            while (this.driveWrapper.isBusy()) this.driveWrapper.update();
        } else if (this.parking == 2) {
            this.driveWrapper.setPose(0, 51 + 24, Math.PI / 2);
            while (this.driveWrapper.isBusy()) this.driveWrapper.update();
        } else {
            this.driveWrapper.setPose(0 - 24, 51, Math.PI / 2);
            while (this.driveWrapper.isBusy()) this.driveWrapper.update();
        }
    }

    public void start() {
        this.thread = new Thread(this, this.threadName);
        this.thread.start();
    }
}
