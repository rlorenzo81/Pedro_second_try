package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Back", group = "Examples")
public class RedAutoBack extends OpMode {
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private DcMotor frontIntake;
    private CRServo rightTrigger;
    private CRServo leftTrigger;
    private DcMotor middleIntake;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(116, 128, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(87, 93, Math.toRadians(223)); // 229x90 y94 It is facing the goal at a 135 degree angle.
    private final Pose scorePose2 = new Pose(91, 83, Math.toRadians(221)); //224x94 y84was 71,74 (220)
    private final Pose pickup1Pose = new Pose(95, 86, Math.toRadians(0)); // y was 86 Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose driveThroughStack1Pose = new Pose(124.5, 86, Math.toRadians(0)); // x127 Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(95, 61, Math.toRadians(0)); // y was 61.5
    private final Pose driveThroughStack2Pose = new Pose(124.5, 59, Math.toRadians(0)); //x=134
    private final Pose scorePose3 = new Pose(91, 84, Math.toRadians(221)); //was 71, 74 (217)
    private final Pose leavingPose = new Pose(95, 59, Math.toRadians(180));
     // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(45, 61, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    //PathConstraints slow = new PathConstraints(0.25, /*maxVel ips*/ 60, /*maxAccel*/ 1.0, /*maxAngVel*/ 1.0);
//toIntakeSlow.getPath(0).setPathConstraints(slow);  o
   // toIntakeSlow.setPathConstraints(slow);
    private Path scorePreload;
    private PathChain grabPickup1,intakeStack1, scorePickup1, grabPickup2, intakeStack2, scorePickup2, grabPickup3, scorePickup3,leaveOutChain;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        intakeStack1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, driveThroughStack1Pose))

                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), driveThroughStack1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(driveThroughStack1Pose, scorePose2))

                .setLinearHeadingInterpolation(driveThroughStack1Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakeStack2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, driveThroughStack2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), driveThroughStack2Pose.getHeading())
                .build();

        scorePickup2= follower.pathBuilder()
                .addPath(new BezierLine(driveThroughStack2Pose, scorePose3))
                .setLinearHeadingInterpolation(driveThroughStack2Pose.getHeading(), scorePose3.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        leaveOutChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose3, leavingPose))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), leavingPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);

                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    intakeTriggerShooterOn();
                    sleepMs(5000); //was 4500, trying to shorten the time after shooting, if this works change it for every other one

                    intakeTriggerShooterOff();
                    sleepMs(200);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intakeOn();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(intakeStack1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intakeOn();
                    sleepMs(300);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    intakeTriggerShooterOn();
                    sleepMs(5000);//5000

                    intakeTriggerShooterOff();
                    sleepMs(200);
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */intakeOn();
                    sleepMs(500);


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(intakeStack2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    sleepMs(300);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    intakeTriggerShooterOn();
                    sleepMs(5000);//5000

                    intakeTriggerShooterOff();
                    sleepMs(200);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(leaveOutChain, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        leftShooter = hardwareMap.get(DcMotorEx.class, "ld");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rd");
        frontIntake = hardwareMap.get(DcMotor.class, "fi");
        rightTrigger = hardwareMap.get(CRServo.class, "rt");
        leftTrigger = hardwareMap.get(CRServo.class, "lt");
        middleIntake=hardwareMap.get(DcMotor.class,"mi");


        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       // rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));
      //  leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }


public void intakeOn(){
    frontIntake.setPower(1);
    middleIntake.setPower(0.5);
}

public void intakeOff(){
    frontIntake.setPower(0);
    middleIntake.setPower(0);
}

public void triggerOff(){
    leftTrigger.setPower(0);
    rightTrigger.setPower(0);

}
public void triggerOn(){
    leftTrigger.setPower(-1);
    rightTrigger.setPower(1);
}
public void shooterOn(){
    leftShooter.setPower(-0.75);
    rightShooter.setPower(0.75);

}

public void shooterOff(){
    leftShooter.setPower(0);
    rightShooter.setPower(0);
    }
    public void intakeTriggerShooterOn(){

       leftShooter.setPower(-0.75);
        rightShooter.setPower(0.75);
        leftTrigger.setPower(-1);
        rightTrigger.setPower(1);
        frontIntake.setPower(1);
        middleIntake.setPower(0.5);
}
    public void intakeTriggerShooterOff(){
       // leftShooter.setPower(0);
      //  rightShooter.setPower(0);
    leftTrigger.setPower(0);
    rightTrigger.setPower(0);
    frontIntake.setPower(0);
    middleIntake.setPower(0);
}
    private void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {
        }
    }
}



