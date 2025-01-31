package legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTLeftAuto3Samp extends LinearOpMode {

    private Servo elbowClaw;
    private DcMotor wrist;

    private JTracking tracker;
    private JArm armPID;
    private SparkFunOTOS.Pose2D pose;

    // constants (for readability)
    final SparkFunOTOS.Pose2D specimenGrabPose = new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, -55, -90);

    final double specimenPlaceX = 32.5;
    final double specimenInitPlaceY = 6;

    final double sampleX = 58;


    @Override
    public void runOpMode() {
        elbowClaw = hardwareMap.get(Servo.class, "elbowClaw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        elbowClaw.scaleRange(0.51, 1);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tracker = new JTracking(this, hardwareMap);
        armPID = new JArm(this, hardwareMap);
        // it's easier to set the back wall to be x = 0 so we only have positive x values
        pose = new SparkFunOTOS.Pose2D(JTracking.robotLength/2, 24-JTracking.robotWidth/2, 0);
        tracker.setPosition(pose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        armPID.start();
        while (opModeIsActive()) {
            // ROB: moves to chamber
            elbowClaw.setPosition(1);
            armPID.setTarget(JArm.specimenPlace);
            sleep(900);
            tracker.moveTo(specimenPlaceX, specimenInitPlaceY, 0, 0.1, 0.5, 0.6);
            elbowClaw.setPosition(1);
            // ROB: clips SPEC PRLD
            tracker.setMotorsMecanum(0.4, 0, 0);
            sleep(800);
            tracker.stopMotors();
            // ROB: opens claw
            elbowClaw.setPosition(0);
            sleep(200);

            /**************************************************************************************/

            // ROB: moves to SAMP 1
            tracker.moveTo(specimenPlaceX-6, specimenInitPlaceY, 0, 1, 0.5, 1.0);
            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(specimenPlaceX-6, 36, 0, 1, 0.5, 1.0);
            tracker.moveTo(sampleX, 34, 0, 1, 0.5, 1.0);
            tracker.moveTo(sampleX, 46, 0, 1, 0.5,1.0);

            /**************************************************************************************/

            // ROB: pushes SAMP 1 to N-ZONE
            tracker.moveTo(18, 50, -7, 1, 0.5,1.0);
            tracker.moveTo(sampleX, 46, 0, 1, 0.5,1.0);
            tracker.moveTo(sampleX, 58, 0, 1, 0.5,1.0);

            /**************************************************************************************/

            // ROB: pushes SAMP 2 to N-ZONE
            tracker.moveTo(18, 58, 0, 1, 0.5, 1.0);
            tracker.moveTo(sampleX, 58, 0, 1, 0.5, 1.0);
            tracker.moveTo(sampleX, 70-JTracking.robotWidth/2, 0, 1, 0.5, 1.0);

            /**************************************************************************************/

            // ROB: pushes SAMP 3 to N-ZONE
            tracker.moveTo(18, 70-JTracking.robotWidth/2, 0, 1, 0.5,1.0);

            /**************************************************************************************/

            // ROB: parks in A-ZONE
            armPID.setTarget(JArm.parkingHeight);
            tracker.moveTo(56, 48, -90, 2, 0.5,0.9); // Robot moves itself to ascent zone
            tracker.moveTo(56, 25, -90, 0.5, 0.5, 0.9); // Robot moves forward into the low bar
            armPID.setTarget(JArm.parkingHeight - 0.1);

            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}