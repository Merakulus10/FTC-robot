
package org.firstinspires.ftc.teamcode;
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

@Autonomous(name = "Blue Auto", group = "Autonomous")
public class blueAuto extends OpMode {
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
    ElapsedTime timer = new ElapsedTime();

    // TODO: MODIFY START HEADING AS NEEDED
    private final Pose startPose = new Pose(20.451127819548873, 122.40601503759397, Math.toRadians(135));

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
        follower.setStartingPose(startPose);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        launcherPower = 0.6; // TODO: Set launcher power
        launcherAccelTime = 3000; // TODO: Set launcher acceleration time in milliseconds
        launchState = -1;
    }

    void launchServo(double power) {
        leftFeeder.setPower(power);
        rightFeeder.setPower(power);
    }
    void startIntake() {
        intake.setPower(1);
        launcher.setPower(-0.2);
        launchServo(-1);
    }
    void stopIntake() {
        intake.setPower(0);
        launcher.setPower(0);
        launchServo(0);
    }
    void launch() {
        if (launchState == 0) { // Start
            stopIntake();
            launcher.setPower(launcherPower);
            timer.reset();
            launchState = 1;
        } else if (launchState == 1) {
            if (timer.milliseconds() >= launcherAccelTime) {
                launchServo(1);
                timer.reset();
                launchState = 2;
            }
        } else if (launchState == 2) {
            if (timer.milliseconds() >= 3000) {
                stopIntake();
                launchState = -1; // Finish
            } else if (timer.milliseconds() % 1000 >= 800) { // 4th step
                intake.setPower(0);
            } else if (timer.milliseconds() % 1000 >= 300) { // 3rd step
                intake.setPower(1);
            } else if (timer.milliseconds() % 1000 >= 100) { // 2nd step
                intake.setPower(0);
            } else { // 1st step
                intake.setPower(-0.5);
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        // Position values for easy editing
        private double[] startPos = {20, 123, Math.toRadians(135)};
        private double[] launchPos = {37, 106, Math.toRadians(135)};
        private double[] intake1StartPos = {51.5, 87, Math.toRadians(180)};
        private double[] intake1EndPos = {20, intake1StartPos[1], intake1StartPos[2]};
        private double[] endPos = {55, 132, Math.toRadians(90)};

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(startPos[0], startPos[1]),
                    new Pose(launchPos[0], launchPos[1])
                )
            ).setLinearHeadingInterpolation(startPos[2], launchPos[2]).build();
            Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(launchPos[0], launchPos[1]),
                    new Pose(intake1StartPos[0], intake1StartPos[1])
                )
            ).setLinearHeadingInterpolation(launchPos[2], intake1StartPos[2]).build();
            Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                     new Pose(intake1StartPos[0], intake1StartPos[1]),
                     new Pose(intake1EndPos[0], intake1EndPos[1])
                )
            ).setLinearHeadingInterpolation(intake1StartPos[2], intake1EndPos[2]).build();
            Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(intake1EndPos[0], intake1EndPos[1]),
                    new Pose(launchPos[0], launchPos[1])
                )
            ).setLinearHeadingInterpolation(intake1EndPos[2], launchPos[2]).build();
            Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(launchPos[0], launchPos[1]),
                    new Pose(endPos[0], endPos[1])
                )
            ).setLinearHeadingInterpolation(launchPos[2], endPos[2]).build();
        }
    }

    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch(pathState){
            case 0:
                launchState = 0; // Start launching pre-loaded artifacts
                follower.followPath(paths.Path1, 0.5, true);
                setPathState(1);
                break;
            case 1:
                if(launchState == -1){
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    startIntake();
                    follower.followPath(paths.Path3, 0.3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    stopIntake();
                    launchState = 0; // Launch first row of collected artifacts
                    follower.followPath(paths.Path4, 0.75, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(launchState == -1){
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                follower.deactivateAllPIDFs(); // Stop all movement
                break;
        }
    }

    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    