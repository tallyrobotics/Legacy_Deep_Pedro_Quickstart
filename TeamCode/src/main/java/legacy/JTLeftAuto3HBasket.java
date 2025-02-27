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
    private DcMotor wrist;

    private JTracking tracker;
    private JArm armPID;
    private SparkFunOTOS.Pose2D pose;

    // constants (for readability)
    final double specPlaceX = 32.5;
    final double specInitPlaceY = 6;

    final double sampX = 36;
    final double initSampY = 24;
    final double sampDisplaceY = 10.5;

    final int wristOutEnc = 1633;

    @Override
    public void runOpMode() {
        specClaw = hardwareMap.get(Servo.class, "elbowClaw");
        sampClaw = hardwareMap.get(Servo.class, "intakeClaw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        specClaw.scaleRange(0, 1);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setTargetPosition(0);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(0.6);

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

            // ROB: moves to and grabs SAMP 1
            tracker.moveTo(specPlaceX-6, specInitPlaceY, 0, 1, 0.5, 1.0);

            armPID.setTarget(JArm.groundGrab);
            tracker.moveTo(sampX, initSampY, 25, 1, 0.5, 0.7);
            sleep(500);
            wrist.setTargetPosition(wristOutEnc);
            sleep(500);
            sampClaw.setPosition(0);
            sleep(500);
            armPID.setTarget(JArm.basketPlace);
            sleep(500);


            /**************************************************************************************/

            // ROB: drops SAMP 1 into BASKET
            tracker.moveTo(22, 22, -135, 1, 0.5,0.7);
            sleep(500);
            sampClaw.setPosition(1);
            sleep(500);

            /**************************************************************************************/

            // ROB: moves to and grabs SAMP 2
            armPID.setTarget(JArm.groundGrab);
            tracker.moveTo(sampX, initSampY+sampDisplaceY, 25, 1, 0.5, 0.7);
            sleep(500);
            wrist.setTargetPosition(wristOutEnc);
            sleep(500);
            sampClaw.setPosition(0);
            sleep(500);
            armPID.setTarget(JArm.basketPlace);
            sleep(500);

            /**************************************************************************************/

            // ROB: drops SAMP 2 into BASKET
            tracker.moveTo(22, 22, -135, 1, 0.5,0.7);
            sleep(500);
            sampClaw.setPosition(1);
            sleep(500);

            /**************************************************************************************/

            // ROB: moves to and grabs SAMP 3
            armPID.setTarget(JArm.groundGrab);
            tracker.moveTo(sampX, initSampY+2*sampDisplaceY, 25, 1, 0.5, 0.7);
            sleep(500);
            wrist.setTargetPosition(wristOutEnc);
            sleep(500);
            sampClaw.setPosition(0);
            sleep(500);
            armPID.setTarget(JArm.basketPlace);
            sleep(500);

            /**************************************************************************************/

            // ROB: drops SAMP 3 into BASKET
            tracker.moveTo(22, 22, -135, 1, 0.5,0.7);
            sleep(500);
            sampClaw.setPosition(1);
            sleep(500);

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