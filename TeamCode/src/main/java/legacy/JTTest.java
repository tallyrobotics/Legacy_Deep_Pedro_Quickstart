package legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous (group = "Auto JTracking")
public class JTTest extends LinearOpMode {
    private Servo elbowClaw;
    private JTracking tracker;
    private JArm armPID;
    private SparkFunOTOS.Pose2D pose;

    @Override
    public void runOpMode() {
        elbowClaw = hardwareMap.get(Servo.class, "elbowClaw");
        elbowClaw.scaleRange(0.51, 1);

        tracker = new JTracking(this, hardwareMap);
        armPID = new JArm(this, hardwareMap);
        // it's easier to set the back wall to be x = 0 so we only have positive x values
        pose = new SparkFunOTOS.Pose2D(0, 0, 0);
        tracker.setPosition(pose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        armPID.start();
        while (opModeIsActive()) {
            // ROB: moves to chamber
            elbowClaw.setPosition(1);
            tracker.moveTo(48*48/51.5, 0, 0, 0.1, 0.5, 0.6);


            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}