/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Thread;

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
public class StarterBotTeleopMecanums extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private CRServo launcher = null;
    private CRServo intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime modifyAttrButtonTimer = new ElapsedTime();

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double launcherPower;
    int launcherAccelTime;
    int launchState;
    boolean modifyAttrButtonPressed;
    boolean intakeOn;

    Telemetry.Item launcherPowerItem;
    Telemetry.Item launcherAccelTimeItem;
    Telemetry.Item launchStateItem;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(CRServo.class, "intake");
        launcher = hardwareMap.get(CRServo.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left");
        rightFeeder = hardwareMap.get(CRServo.class, "right");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        launcher.setDirection(CRServo.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        launcherPower = 0.7; // 0 to 1 power
        launcherAccelTime = 3000; // milliseconds
        launchState = 0; // 0: idle, 1: accelerating launcher, 2: feeding artifacts
        modifyAttrButtonPressed = false; // to prevent multiple changes per button press

        telemetry.setAutoClear(false);
        launcherPowerItem = telemetry.addData("Launcher Power", "%.2f", launcherPower);
        launcherAccelTimeItem = telemetry.addData("Launcher Accel Time", "%d ms", launcherAccelTime);
        launchStateItem = telemetry.addData("Launch State", "Idle");
    }

    @Override
    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (launchState == 1 && timer.milliseconds() >= launcherAccelTime) { // launcher has finished accelerating
            launchState = 2;
            launch();
        } else if (gamepad1.b) { // start launch sequence
            launch();
        }
        if (gamepad1.left_trigger > 0) {
            intake.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0) {
            intake.setPower(-gamepad1.right_trigger);
        } else if (gamepad1.a) { // run intake
            if (intakeOn) {
                stopIntake();
            } else {
                intake();
            }
            intake();
        } else {
            stopIntake();
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_up || gamepad1.dpad_down) {
            if (modifyAttrButtonPressed && modifyAttrButtonTimer.milliseconds() < 500) {
                return;
            }
            modifyAttrButtonTimer.reset();
            if (gamepad1.left_bumper) {
                launcherPower = Math.max(launcherPower - 0.05, 0);
                launcherPowerItem.setValue("%.2f", launcherPower);
            }
            if (gamepad1.right_bumper) {
                launcherPower = Math.min(launcherPower + 0.05, 1);
                launcherPowerItem.setValue("%.2f", launcherPower);
            }
            if (gamepad1.dpad_up) {
                launcherAccelTime += 100;
            }
            if (gamepad1.dpad_down) {
                launcherAccelTime = Math.max(launcherAccelTime - 100, 0);
            }
            launcherPowerItem.setValue("%.2f", launcherPower);
            launcherAccelTimeItem.setValue("%d ms", launcherAccelTime);
            modifyAttrButtonPressed = true;
        } else {
            modifyAttrButtonPressed = false;
        }
    }

    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launchServo(double power) {
        leftFeeder.setPower(power);
        rightFeeder.setPower(power);
    }
    void intake() {
        if (launchState != 0) {
            return;
        }
        intake.setPower(1);
        launcher.setPower(-0.2);
        launchServo(-1);
    }
    void stopIntake() {
        intake.setPower(0);
        if (launchState == 0) { // Only reset if launcher idle
            launcher.setPower(0);
        }
        launchServo(0);
    }

    void launch() {
        if (launchState == 0) {
            stopIntake();
            launcher.setPower(launcherPower);
            timer.reset();
            launchState = 1;
            launchStateItem.setValue("Accelerating");
        } else if (launchState == 2) {
            mecanumDrive(0, 0, 0);
            launchServo(1);
            for (int i=0; i<3; i++) {
                launchStateItem.setValue("Feed loop %d", i+1);
                telemetry.update();
                intake.setPower(-0.5);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {}

                intake.setPower(0);
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {}

                intake.setPower(1);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {}

                intake.setPower(0);
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {}
            }
            launcher.setPower(0);
            launchServo(0);
            launchState = 0;
            launchStateItem.setValue("Idle");
        }
    }
}