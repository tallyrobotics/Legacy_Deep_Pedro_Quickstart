package legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "Auto JTracking")
public class JATest extends LinearOpMode {
    JArm armPID;
boolean test = true;
    @Override
    public void runOpMode() {
        armPID = new JArm(this, hardwareMap);
        waitForStart();
        armPID.start();// .run();
        while (opModeIsActive()) {
//            sleep(10000);
            armPID.setTarget(JArm.specimenPlace);
            sleep(15000);
            armPID.setTarget(JArm.specimenGrab);
            while (opModeIsActive()) {
                sleep(1000);
            }
        }
    }
}
