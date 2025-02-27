package legacy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "pedroRightAuto", group = "Testing")
//@Disabled
public class pedroRightAuto extends OpMode
{
    private final int ECLAW_OPEN = 0;
    private final int ECLAW_CLOSED = 1;
    private Telemetry telemetryA;
    private DcMotor wrist;
    private Servo elbowClaw;
    private pedroArm armPID;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;

    private enum PATHSTATE
    {
        HANGPRELOAD,
        HANGDONE
    }

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(10, 57, Math.toRadians(0));
    /**
     * Specimen Place Pose; change X component to place Specimen on different parts of High Chamber
     */
    private final Pose firstSpecPose = new Pose(38, 63, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain scorePreload1, pushSamp1, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
    public void buildPaths()
    {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(firstSpecPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), firstSpecPose.getHeading());

        scorePreload1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(firstSpecPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstSpecPose.getHeading())
//                .setZeroPowerAccelerationMultiplier(2.0)
                .addParametricCallback(0.99, () -> {elbowClaw.setPosition(ECLAW_OPEN);})
                .build();


        pushSamp1 = follower.pathBuilder()
            .addPath(
            // Line 2
            new BezierCurve(
                    new Point(35.000, 63.000, Point.CARTESIAN),
                    new Point(1.300, 33.800, Point.CARTESIAN),
                    new Point(50.000, 35.500, Point.CARTESIAN)
            )
    )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
                    // Line 3
                    new BezierLine(
                            new Point(50.000, 35.500, Point.CARTESIAN),
                            new Point(50.000, 27.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
                    // Line 4
                    new BezierLine(
                            new Point(50.000, 27.000, Point.CARTESIAN),
                            new Point(17.000, 27.000, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(17.000, 27.000, Point.CARTESIAN),
                                new Point(28.300, 29.000, Point.CARTESIAN),
                                new Point(69.732, 31.589, Point.CARTESIAN),
                                new Point(61.834, 14.114, Point.CARTESIAN),
                                new Point(27.893, 15.795, Point.CARTESIAN),
                                new Point(17.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(17.000, 16.000, Point.CARTESIAN),
                                new Point(24.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(24.000, 24.000, Point.CARTESIAN),
                                new Point(9.500, 20.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
//        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        park = new Path(new BezierLine(new Point(firstSpecPose), new Point(startPose)));
        park.setLinearHeadingInterpolation(firstSpecPose.getHeading(), startPose.getHeading());
    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate()
    {
        switch (pathState)
        {
            case 0: // Wait for arm to raise a little
                if (pathTimer.getElapsedTimeSeconds() > 1.0)
                {
                    setPathState(1);
                }
                break;
            case 1: // Drive forward to hang specimen
                follower.setMaxPower(0.5);
                follower.followPath(scorePreload1, true);
                setPathState(2);
                break;
//                /* You could check for
//                - Follower State: "if(!follower.isBusy() {}"
//                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//                - Robot Position: "if(follower.getPose().getX() > 36) {}"
//                */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1,true);
//                    setPathState(2);
//                }
//                break;
            case 2: // Lower arm after releasing specimen
                if (pathTimer.getElapsedTimeSeconds() > 2.0)
                {
                    armPID.setTarget(pedroArm.specimenPlace - 20);
                    setPathState(3);
                }
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if(!follower.isBusy())
//                {
//                    elbowClaw.setPosition(ECLAW_OPEN);
////                    /* Grab Sample */
////
////                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
////                    follower.followPath(scorePickup1,true);
//                    setPathState(3);
//                }
                break;
            case 3: // Allow servo time to open
                if (pathTimer.getElapsedTimeSeconds() > 3.0)
                {
                    armPID.setTarget(pedroArm.specimenGrab);
                    follower.followPath(pushSamp1);
                    setPathState(4);
//                    armPID.setTarget(pedroArm.specimenPlace - 20);
//                    setPathState(3);
                }
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2,true);
//                    setPathState(4);
//                }
                break;
            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup2,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup3, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(park,true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy())
                {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState)
    {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop()
    {
        armPID.run();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub
//        follower.telemetryDebug(telemetryA);
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
    public void init()
    {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        wrist = hardwareMap.get(DcMotor.class, "wrist");
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elbowClaw = hardwareMap.get(Servo.class, "elbowClaw");
        elbowClaw.scaleRange(0, 1.0);
        elbowClaw.setPosition(ECLAW_CLOSED);

        armPID = new pedroArm(this, hardwareMap);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop()
    {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
        wrist.setPower(-0.2);
//        armPID.start();
        armPID.setTarget(pedroArm.specimenPlace);
        elbowClaw.setPosition(ECLAW_CLOSED);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop()
    {
        armPID.end();
    }
}