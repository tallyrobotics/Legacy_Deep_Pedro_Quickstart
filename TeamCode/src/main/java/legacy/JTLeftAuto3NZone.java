package legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTLeftAuto3NZone extends LinearOpMode {

    private Servo specClaw;
    private DcMotor wrist;

    private JTracking tracker;
    private JArm armPID;
    private SparkFunOTOS.Pose2D pose;

    // constants (for readability)
    final SparkFunOTOS.Pose2D specimenGrabPose = new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, -55, -90);

    final double specPlaceX = 32.5;
    final double specInitPlaceY = 8;

    final double sampleX = 58;

    @Override
    public void runOpMode() {
        specClaw = hardwareMap.get(Servo.class, "elbowClaw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        specClaw.scaleRange(0, 1);
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
            specClaw.setPosition(0);
            armPID.setTarget(JArm.specimenPlace);
            sleep(1000);
            tracker.moveTo(specPlaceX, specInitPlaceY, 0, 1, 0.5, 0.8);
            specClaw.setPosition(0);

            // ROB: clips SPEC PRLD
            tracker.setMotorsMecanum(0.8, 0, 0);
            sleep(400);
            tracker.stopMotors();
            specClaw.setPosition(1);
            sleep(200);

            /**************************************************************************************/

            // ROB: moves to SAMP 1
            tracker.moveTo(specPlaceX-4, specInitPlaceY, 0, 1, 0.5, 1.0);
            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(specPlaceX-4, 36, 0, 1, 0.5, 1.0);
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
            tracker.moveTo(sampleX, 71-JTracking.robotWidth/2, 0, 1, 0.5, 1.0);

            /**************************************************************************************/

            // ROB: pushes SAMP 3 to N-ZONE
            tracker.moveTo(18, 71-JTracking.robotWidth/2, 0, 1, 0.5,1.0);

            /**************************************************************************************/

            // ROB: parks in A-ZONE
            armPID.setTarget(JArm.parkingHeight + 2);
            tracker.moveTo(56, 48, -90, 2, 0.5,0.9); // Robot moves itself to ascent zone
            tracker.moveTo(56, 25, -90, 0.5, 0.5, 0.9); // Robot moves forward into the low bar
            armPID.setTarget(JArm.parkingHeight - 2);

            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}