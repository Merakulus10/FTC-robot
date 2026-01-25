package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@TeleOp
public class TutorialAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        //Start position_End position
        //Drive > Movement state
        //Shoot > Attempt to score the artifact

        Drive_startpos_Shoot_pos,
        Shoot_preload,
        Drive_Shootpos_Endpos
    }

     PathState pathState;

    private final Pose startPose = new Pose(19.597633136094675, 122.69822485207101, Math.toRadians(138));
    private final Pose shootPose = new Pose(55.14116652578191, 87.88503803888419, Math.toRadians(138));

    private final Pose endPose = new Pose(74.91293322062553, 99.99154691462384, Math.toRadians(90));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths(){
        // put in coordinates for starting pose > ending pose

        driveStartPosShootPos = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose)).setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading()).build();
        driveShootPosEndPos = follower.pathBuilder().addPath(new BezierLine(shootPose, endPose)).setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading()).build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case Drive_startpos_Shoot_pos:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.Shoot_preload); // reset the timer and make new state
                break;

            case Shoot_preload:
                //check is follower done its path?
                // and check that 5 seconds has elapsed
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    //add flywheel shooter

                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.Drive_Shootpos_Endpos);
                }
                break;

            case Drive_Shootpos_Endpos:
                if(!follower.isBusy()){
                    telemetry.addLine("Done all paths");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.Drive_startpos_Shoot_pos;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
