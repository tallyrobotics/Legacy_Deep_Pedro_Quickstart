package legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTLeftAuto3HBasket extends LinearOpMode {

    private Servo specClaw;
    private Servo sampClaw;
    private Servo rotator;
    private CRServo extender;
    private DcMotor wrist;

    private JTracking tracker;
    private JArm armPID;
    private SparkFunOTOS.Pose2D pose;

    // constants (for readability)
    SparkFunOTOS.Pose2D sampDropPose = new SparkFunOTOS.Pose2D(18, 30, 135);

    final double armGrabLDist = 30;

    final int wristOutEnc = -1633;

    @Override
    public void runOpMode() {
        sampClaw = hardwareMap.get(Servo.class, "intakeClaw");
        rotator = hardwareMap.get(Servo.class, "intakeRotate");
        extender = hardwareMap.get(CRServo.class, "extender");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        sampClaw.scaleRange(0, 1);
        rotator.scaleRange(0, 1);
        
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setTargetPosition(0);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(0.8);

        tracker = new JTracking(this, hardwareMap);
        armPID = new JArm(this, hardwareMap);
        // it's easier to set the back wall to be x = 0 so we only have positive x values
        pose = new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, 24+JTracking.robotLength/2, 90);
        tracker.setPosition(pose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        armPID.start();
        while (opModeIsActive()) {

            // ROB: moves to basket 
            sampClaw.setPosition(0);
            armPID.setTarget(JArm.basketPlace);
            wrist.setTarget(wristOutEnc);
            extender.setPower(-1);
            sleep(3100);
            tracker.moveToPose(sampDropPose, 1, 0.5, 0.7);
            sampClaw.setPosition(0);

            // ROB: drops SAMP PRLD
            sampClaw.setPosition(1);
            sleep(400);

            /**************************************************************************************/

            // ROB: grabs SAMP 1
            armPID.setTarget(JArm.basketGrab);
            tracker.moveTo(46-armGrabLDist, 48, 0, 0.1, 0.5, 0.7);
            sampClaw.setPosition(0);
            sleep(400);

            /**************************************************************************************/

            // ROB: moves to basket 
            armPID.setTarget(JArm.basketPlace);
            tracker.moveToPose(sampDropPose, 1, 0.5, 0.7);
            sampClaw.setPosition(0);

            // ROB: drops SAMP 1
            sampClaw.setPosition(1);
            sleep(400);

            /**************************************************************************************/

            // ROB: grabs SAMP 2

            armPID.setTarget(JArm.basketGrab);
            tracker.moveTo(46-armGrabLDist, 59, 0, 0.1, 0.5, 0.7);
            sampClaw.setPosition(0);
            sleep(400);

            /**************************************************************************************/

            // ROB: moves to basket 
            armPID.setTarget(JArm.basketPlace);
            tracker.moveToPose(sampDropPose, 1, 0.5, 0.7);
            sampClaw.setPosition(0);

            // ROB: drops SAMP 2
            sampClaw.setPosition(1);
            sleep(400);

            /**************************************************************************************/

            // ROB: parks in A-ZONE
            armPID.setTarget(JArm.parkingHeight + 5);
            tracker.moveTo(56, 48, -90, 2, 0.5,0.9); // Robot moves itself to ascent zone
            tracker.moveTo(56, 25, -90, 0.5, 0.5, 0.9); // Robot moves forward into the low bar
            armPID.setTarget(JArm.parkingHeight - 5);

            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}