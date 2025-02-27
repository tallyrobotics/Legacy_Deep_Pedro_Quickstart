package legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTRightAuto3Spec extends LinearOpMode {
    private Servo specClaw;

    private JTracking tracker;
    private JArm armPID;
    private SparkFunOTOS.Pose2D pose;

    // constants (for readability)
    final SparkFunOTOS.Pose2D specGrabPose = new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, -57.25, -90);

    final double specPlaceX = 32.5;
    final double specInitPlaceY = -8;
    final double specDisplaceY = 3.5;

    final double sampX = 58;

    @Override
    public void runOpMode() {
        specClaw = hardwareMap.get(Servo.class, "elbowClaw");

        specClaw.scaleRange(0, 1);

        tracker = new JTracking(this, hardwareMap);
        armPID = new JArm(this, hardwareMap);
        // it's easier to set the back wall to be x = 0 so we only have positive x values
        pose = new SparkFunOTOS.Pose2D(JTracking.robotLength/2, -24+JTracking.robotWidth/2, 0);
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
            tracker.moveTo(specPlaceX-4, specInitPlaceY, 0, 3, 0.5, 1.0);
            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(specPlaceX-4, -36, 0, 3, 0.5, 1.0);
            tracker.moveTo(sampX, -36, 0, 3, 0.5, 1.0);
            tracker.moveTo(sampX, -46, 0, 1, 0.5,1.0);

            // ROB: pushes SAMP 1 to O-ZONE
            tracker.moveTo(18, -46, 0, 3, 0.5,1.0);

            // HP: SAMP 1 --> SPEC 1
            tracker.moveTo(sampX, -46, 0, 3, 0.5,1.0);
            tracker.moveTo(sampX, -56, 0, 1, 0.5,1.0);

            // ROB: pushes SAMP 2 to O-ZONE
            tracker.moveTo(18, -56, 0, 3, 0.5,1.0);

            // HP: SAMP 2 --> SPEC 2
            tracker.moveTo(24, -36, -90, 3, 0.5, 0.8);

            // ROB: squares with back wall
            // HP: Aligns SPEC 1
            tracker.setMotorsMecanum(0.1, 0.6, 0);
            sleep(1100);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, pose.y, -90));

            // ROB: grabs SPEC 1
            tracker.moveToPose(specGrabPose, 0.5, 0.5, 0.5);
            specClaw.setPosition(0);
            sleep(400);

            // ROB: backs away
            armPID.setTarget(JArm.specimenGrab + 10);
            tracker.moveTo(JTracking.robotWidth/2, -42, -90, 1, 0.5, 1.0);

            /**************************************************************************************/

            // ROB: moves to chamber
            // HP: aligns SPEC 2
            armPID.setTarget(JArm.specimenPlace);
            tracker.moveTo(specPlaceX-4, specInitPlaceY + specDisplaceY, 0, 1, 0.5, 1.0);
            tracker.moveTo(specPlaceX, specInitPlaceY + specDisplaceY, 0, 1, 0.5, 0.8);

            // ROB: clips SPEC 1
            tracker.setMotorsMecanum(0.8, 0, 0);
            sleep(400);
            tracker.stopMotors();
            specClaw.setPosition(1);
            sleep(200);

            /**************************************************************************************/

            // ROB: moves to prepare for squaring
            tracker.moveTo(22, specInitPlaceY + specDisplaceY, 0, 3, 0.5, 1.0);
            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(12, -12, -90, 3, 0.5, 1.0);

            // ROB: squares with back wall
            tracker.setMotorsMecanum(0.8, 0.6, 0);
            sleep(900);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, pose.y, -90));

            // ROB: grabs SPEC 2
            tracker.moveToPose(specGrabPose, 0.5, 0.5, 0.5);
            specClaw.setPosition(0);
            sleep(400);
            // back away
            armPID.setTarget(JArm.specimenGrab + 10);
            tracker.moveTo(JTracking.robotWidth/2, -42, -90, 1, 0.5, 1.0);

            /**************************************************************************************/

            // ROB: moves to chamber
            armPID.setTarget(JArm.specimenPlace);
            tracker.moveTo(specPlaceX-4, specInitPlaceY + specDisplaceY*2, 0, 1, 0.5, 1.0);
            tracker.moveTo(specPlaceX, specInitPlaceY + specDisplaceY *2, 0, 1, 0.5, 0.8);

            // ROB: clips SPEC 2
            tracker.setMotorsMecanum(0.8, 0, 0);
            sleep(400);
            tracker.stopMotors();
            specClaw.setPosition(1);
            sleep(200);

            /**************************************************************************************/

            // ROB: parks in O-ZONE
            armPID.setTarget(JArm.specimenPlace-12);
            tracker.moveTo(12, -42, 0, 1, 0.5, 1.0);

            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}