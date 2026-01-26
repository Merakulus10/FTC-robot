
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Colourblind Auto", group = "Autonomous")
public class colourblindAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, opmodeTimer;

    private CRServo launcher = null;
    private CRServo intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    double launcherPower;
    int launcherAccelTime;
    int launchState;
    boolean isRed;
    boolean initLoopButtonPressed;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init_loop() {
        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.b) {
            if (initLoopButtonPressed) {
                return;
            }
            if (gamepad1.left_bumper) {
                launcherPower = Math.max(launcherPower - 0.01, 0);
            }
            if (gamepad1.right_bumper) {
                launcherPower = Math.min(launcherPower + 0.01, 1);
            }
            if (gamepad1.dpad_up) {
                launcherAccelTime += 100;
            }
            if (gamepad1.dpad_down) {
                launcherAccelTime = Math.max(launcherAccelTime - 100, 0);
            }
            if (gamepad1.b) {
                isRed = !isRed;
                paths = new Paths(follower, isRed); // Rebuild paths
                follower.setStartingPose(new Pose(paths.startPos[0], paths.startPos[1], paths.startPos[2]));
            }
            initLoopButtonPressed = true;
        } else {
            initLoopButtonPressed = false;
        }

        /*telemetry.addData("Launcher Power", "%.2f", launcherPower);
        telemetry.addData("Launcher Accel Time", "%d ms", launcherAccelTime);
        telemetry.addData("Alliance Colour", isRed ? "Red" : "Blue");*/
        panelsTelemetry.debug("Launcher Power: " + String.format("%.2f", launcherPower));
        panelsTelemetry.debug("Launcher Accel Time: " + String.format("%d ms", launcherAccelTime));
        panelsTelemetry.debug("Alliance Colour: " + (isRed ? "Red" : "Blue"));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init() {
        intake = hardwareMap.get(CRServo.class, "intake");
        launcher = hardwareMap.get(CRServo.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left");
        rightFeeder = hardwareMap.get(CRServo.class, "right");

        launcher.setDirection(CRServo.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        // Default values
        launcherPower = 0.6; // 0.55 for full battery, 0.6 for half battery
        launcherAccelTime = 3000;
        isRed = false; // true for Red, false for Blue
        paths = new Paths(follower, false);
        follower.setStartingPose(new Pose(paths.startPos[0], paths.startPos[1], paths.startPos[2]));

        launchState = -1;
        initLoopButtonPressed = false;
    }

    void launchServo(double power) {
        leftFeeder.setPower(power);
        rightFeeder.setPower(power);
    }

    void startIntake() {
        intake.setPower(1);
        if (launchState == -1) {
            launcher.setPower(-0.2);
        }
        launchServo(-1);
    }

    void stopIntake() {
        intake.setPower(0);
        if (launchState == -1) {
            launcher.setPower(0);
        }
        launchServo(0);
    }

    void launch() {
        if (launchState == 0) { // Start
            launcher.setPower(launcherPower);
            timer.reset();
            launchState = 1;
        } else if (launchState == 1) {
            if (timer.milliseconds() >= launcherAccelTime) {
                launchServo(1);
                timer.reset();
                launchState = 2;
            } else if (timer.milliseconds() >= (launcherAccelTime - 2000)) {
                stopIntake();
            }
        } else if (launchState == 2) {
            if (timer.milliseconds() >= 3000) {
                launchState = -1; // Finish
                stopIntake();
            } else if (timer.milliseconds() % 1000 >= 800) { // 4th step
                intake.setPower(0);
            } else if (timer.milliseconds() % 1000 >= 300) { // 3rd step
                intake.setPower(1);
            } else if (timer.milliseconds() % 1000 >= 100) { // 2nd step
                intake.setPower(0);
            } else { // 1st step
                intake.setPower(-0.8);
            }
        }
    }

    @Override
    public void loop() {
        launch(); // Update launcher state machine
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public final PathChain Path1;
        public final PathChain Path2;
        public final PathChain Path3;
        public final PathChain Path4;
        public final PathChain Path5;

        // TODO: Edit positions if needed
        // Position values for easy editing - Blue Alliance values used as default
        private final double[] colourblindStartPos = { 20, 123, Math.toRadians(135) };
        private final double[] colourblindLaunchPos = { 37, 106, Math.toRadians(135) };
        private final double[] colourblindIntake1StartPos = { 50, 87, Math.toRadians(180) };
        private final double[] colourblindIntake1EndPos = { 22, colourblindIntake1StartPos[1], colourblindIntake1StartPos[2] };
        private final double[] colourblindEndPos = { 55, 127, Math.toRadians(90) };

        public final double[] startPos;
        public final double[] launchPos;
        public final double[] intake1StartPos;
        public final double[] intake1EndPos;
        public final double[] endPos;

        public Paths(Follower follower, boolean isRed) {
            if (isRed) { // Red Alliance - mirror positions
                startPos = new double[]{ 144 - colourblindStartPos[0], colourblindStartPos[1], Math.PI - colourblindStartPos[2] };
                launchPos = new double[]{ 144 - colourblindLaunchPos[0], colourblindLaunchPos[1], Math.PI - colourblindLaunchPos[2] };
                intake1StartPos = new double[]{ 144 - colourblindIntake1StartPos[0], colourblindIntake1StartPos[1], Math.PI - colourblindIntake1StartPos[2] };
                intake1EndPos = new double[]{ 144 - colourblindIntake1EndPos[0], colourblindIntake1EndPos[1], Math.PI - colourblindIntake1EndPos[2] };
                endPos = new double[]{ 144 - colourblindEndPos[0], colourblindEndPos[1], Math.PI - colourblindEndPos[2] };
            } else {
                startPos = colourblindStartPos;
                launchPos = colourblindLaunchPos;
                intake1StartPos = colourblindIntake1StartPos;
                intake1EndPos = colourblindIntake1EndPos;
                endPos = colourblindEndPos;
            }
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(startPos[0], startPos[1]),
                            new Pose(launchPos[0], launchPos[1])))
                    .setLinearHeadingInterpolation(startPos[2], launchPos[2]).build();
            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(launchPos[0], launchPos[1]),
                            new Pose(intake1StartPos[0], intake1StartPos[1])))
                    .setLinearHeadingInterpolation(launchPos[2], intake1StartPos[2]).build();
            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(intake1StartPos[0], intake1StartPos[1]),
                            new Pose(intake1EndPos[0], intake1EndPos[1])))
                    .setLinearHeadingInterpolation(intake1StartPos[2], intake1EndPos[2]).build();
            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(intake1EndPos[0], intake1EndPos[1]),
                            new Pose(launchPos[0], launchPos[1])))
                    .setLinearHeadingInterpolation(intake1EndPos[2], launchPos[2]).build();
            Path5 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(launchPos[0], launchPos[1]),
                            new Pose(endPos[0], endPos[1])))
                    .setLinearHeadingInterpolation(launchPos[2], endPos[2]).build();
        }
    }

    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:
                launchState = 0; // Start launching pre-loaded artifacts
                startIntake();
                follower.followPath(paths.Path1, 0.5, true);
                setPathState(2);
                break;
            case 1: // TODO: UNUSED
                if (pathTimer.getElapsedTime() > (launcherAccelTime - 1000)) {
                    stopIntake();
                    setPathState(2);
                }
                break;
            case 2:
                if (launchState == -1) {
                    follower.followPath(paths.Path2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(paths.Path3, 0.3, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    launchState = 0; // Launch first row of collected artifacts
                    follower.followPath(paths.Path4, 0.75, true);
                    setPathState(6);
                }
                break;
            case 5: // TODO: UNUSED
                if (pathTimer.getElapsedTime() > (launcherAccelTime - 1000)) {
                    stopIntake();
                    setPathState(6);
                }
                break;
            case 6:
                if (launchState == -1) {
                    follower.followPath(paths.Path5, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.deactivateAllPIDFs(); // Stop all movement
                    setPathState(-1); // End of auto
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
