package legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTRightAuto3Spec extends LinearOpMode {
    private Servo elbowClaw;
    private DcMotor wrist;

    private JTracking tracker;
    private JArm armPID;
    private SparkFunOTOS.Pose2D pose;

    // constants (for readability)
    final SparkFunOTOS.Pose2D specimenGrabPose = new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, -58.75, -90);

    final double specimenPlaceX = 32.5;
    final double specimenInitPlaceY = -6;
    final double specimenPlaceYOffset = 3;

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
        pose = new SparkFunOTOS.Pose2D(JTracking.robotLength/2, -24+JTracking.robotWidth/2, 0);
        tracker.setPosition(pose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        armPID.start();
        while (opModeIsActive()) {
            // ROB: moves to chamber
            elbowClaw.setPosition(1);
            armPID.setTarget(JArm.specimenPlace);
            sleep(1050);
            tracker.moveTo(specimenPlaceX, specimenInitPlaceY, 0, 0.1, 0.5, 0.6);
            elbowClaw.setPosition(1);

            // ROB: clips SPEC PRLD
            tracker.setMotorsMecanum(0.4, 0, 0);
            sleep(800);
            tracker.stopMotors();
            elbowClaw.setPosition(0);
            sleep(150);
            armPID.setTarget(JArm.specimenPlace - 20);
            sleep(200);


            /**************************************************************************************/

            // ROB: moves to SAMP 1
            tracker.moveTo(specimenPlaceX-6, specimenInitPlaceY, 0, 1, 0.5, 1.0);
            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(specimenPlaceX-6, -36, 0, 1, 0.5, 1.0);
            tracker.moveTo(sampleX, -36, 0, 1, 0.5, 1.0);
            tracker.moveTo(sampleX, -45, 0, 1, 0.5,1.0);

            // ROB: pushes SAMP 1 to O-ZONE
            tracker.moveTo(18, -45, 0, 1, 0.5,1.0);

            // HP: SAMP 1 --> SPEC 1
            tracker.moveTo(sampleX, -45, 0, 1, 0.5,1.0);
            tracker.moveTo(sampleX, -59, 0, 1, 0.5,1.0);

            // ROB: pushes SAMP 2 to O-ZONE
            tracker.moveTo(18, -59, 0, 1, 0.5,1.0);

            // HP: SAMP 2 --> SPEC 2
            tracker.moveTo(24, -36, -90, 1, 0.5, 0.6);

            // ROB: squares with back wall
            // HP: Aligns SPEC 1
            tracker.setMotorsMecanum(0, 0.5, 0);
            sleep(1300);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, pose.y, -90));

            // ROB: grabs SPEC 1
            tracker.moveToPose(specimenGrabPose, 0.1, 0.5, 0.6);
            elbowClaw.setPosition(1);
            sleep(500);

            // ROB: backs away
            tracker.moveTo(JTracking.robotWidth/2, -42, -90, 1, 0.5, 0.9);

            /**************************************************************************************/

            // ROB: moves to chamber
            // HP: aligns SPEC 2
            armPID.setTarget(JArm.specimenPlace);
            tracker.moveTo(specimenPlaceX-6, specimenInitPlaceY + specimenPlaceYOffset, 0, 1, 0.5, 0.9);
            tracker.moveTo(specimenPlaceX, specimenInitPlaceY + specimenPlaceYOffset, 0, 0.1, 0.5, 0.6);

            // ROB: clips SPEC 1
            tracker.setMotorsMecanum(0.4, 0, 0);
            sleep(800);
            tracker.stopMotors();
            elbowClaw.setPosition(0);
            sleep(200);

            /**************************************************************************************/

            // ROB: moves to prepare for squaring
            tracker.moveTo(22, specimenInitPlaceY + specimenPlaceYOffset, 0, 1, 0.5, 0.7);
            armPID.setTarget(JArm.specimenGrab);
            tracker.moveTo(12, -12, -90, 1, 0.5, 0.9);

            // ROB: squares with back wall
            tracker.setMotorsMecanum(0.5, 0.5, 0);
            sleep(1000);
            tracker.stopMotors();
            pose = tracker.getPosition();
            tracker.setPosition(new SparkFunOTOS.Pose2D(JTracking.robotWidth/2, pose.y, -90));

            // ROB: grabs SPEC 2
            tracker.moveToPose(specimenGrabPose, 0.1, 0.5, 0.6);
            elbowClaw.setPosition(1);
            sleep(500);
            // back away
            tracker.moveTo(JTracking.robotWidth/2, -42, -90, 1, 0.5, 0.9);

            /**************************************************************************************/

            // ROB: moves to chamber
            armPID.setTarget(JArm.specimenPlace);
            tracker.moveTo(specimenPlaceX-6, specimenInitPlaceY + specimenPlaceYOffset*2, 0, 1, 0.5, 0.9);
            tracker.moveTo(specimenPlaceX, specimenInitPlaceY + specimenPlaceYOffset*2, 0, 0.1, 0.5, 0.6);

            // ROB: clips SPEC 2
            tracker.setMotorsMecanum(0.4, 0, 0);
            sleep(1000);
            tracker.stopMotors();
            elbowClaw.setPosition(0);
            sleep(200);

            /**************************************************************************************/

            // ROB: parks in O-ZONE
            tracker.moveTo(12, -42, 0, 1, 0.5, 0.9);

            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}