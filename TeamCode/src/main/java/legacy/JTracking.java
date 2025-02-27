package legacy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class JTracking {
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor arm;
    private Servo claw;

    private Telemetry telemetryAll;
    private SparkFunOTOS otos;

    // tinybot
//    public double robotWidth = 8.75;

    // regular bot
    static double robotWidth = 13.8125;
    static double robotLength = 16.6875;

    // tune these values to the point where moveFieldCentric(1, 1, 0, 0.2, 0) moves it exactly diagonally to the top-right
    final double forwardFactor = 1.0;
    final double strafeFactor = 1.0;

//    final double posErrorTolerance = 0.05;
//    final double headingErrorTolerance = 0.5;

    final double position_p = 0.10;
    final double position_d = 0.20;

    final double heading_p = 0.06;
    final double heading_d = 0.08;

    // we add minimum powers to prevent it from halting and never reaching the target.
    // at 0.05 movement power, it can pretty much stop as soon as it wants to.
    final double minPower = 0.05;

    final double maxYaw = 0.60;
    final double minYaw = 0.00;

    public JTracking(LinearOpMode initOpMode, HardwareMap initHardwareMap) {
        opMode = initOpMode;
        hardwareMap = initHardwareMap;

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // tinybot
//        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));

        // regular bot
        otos.setOffset(new SparkFunOTOS.Pose2D(-1.375, 0, 270));

        // tinybot
//        otos.setLinearScalar(1.11);
//        otos.setAngularScalar(0.9675/*1.0*/);

        // regular bot
        otos.setLinearScalar(1.0118583); //0.99856
        otos.setAngularScalar(1.000); //0.9798

        // initialization
        otos.calibrateImu();
        otos.resetTracking();

        telemetryAll = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryAll.addData("Scalar", otos.getLinearScalar());
        telemetryAll.addData("Status", "Initialized");
        telemetryAll.update();
    }

    // angle of 0 is pointing in +X direction, angles increase COUNTERCLOCKWISE

    public void setPosition(SparkFunOTOS.Pose2D pose) {
        otos.setPosition(pose);
        telemetryAll.addLine("Position Updated");
        telemetryAll.addData("x", pose.x);
        telemetryAll.addData("y", pose.y);
        telemetryAll.update();
    }

    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

    public void setMotors(double bl, double br, double fl, double fr) {
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
    }

    public void stopMotors() {
        setMotors(0,0,0,0);
    }

    public void setMotorsMecanum(double axial, double lateral, double yaw) {
        // again, a positive yaw makes the robot rotate COUNTERCLOCKWISE
        setMotors(axial - lateral - yaw, axial + lateral + yaw, axial + lateral - yaw, axial - lateral + yaw);
    }

    public void moveFieldCentric(double dx, double dy, double heading, double power, double yaw) {
        // strafing often requires more power than going forward, so tweaking the forward and strafe factors may be necessary.
        double scaledX = dx * forwardFactor;
        double scaledY = dy * strafeFactor;

        double d = Math.sqrt(scaledX*scaledX + scaledY*scaledY);

        double normalizedX = scaledX / d;
        double normalizedY = scaledY / d;

        // if heading = 0:
        // axial points in x direction
        // lateral points in -y direction

        double lateral = normalizedX * Math.sin(Math.toRadians(heading)) - normalizedY * Math.cos(Math.toRadians(heading));
        double axial   = normalizedX * Math.cos(Math.toRadians(heading)) + normalizedY * Math.sin(Math.toRadians(heading));

        lateral *= power;
        axial *= power;

        setMotorsMecanum(axial, lateral, yaw);
    }

    public double getError(double targetX, double targetY, double currentX, double currentY) {
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;

        return Math.sqrt(errorX*errorX + errorY*errorY);
    }

    public double minimizeAngle(double angle) {
        return ((angle + 180.0) % 360.0) - 180.0;
    }

    public double constrainPower(double min, double max, double power) {
        double absPower = Math.abs(power);
        double sign = Math.signum(power);

        return sign * Math.min(Math.max(absPower, min), max);
    }

    public void moveToPose(SparkFunOTOS.Pose2D pose, double posErrorTolerance, double headingErrorTolerance, double maxPower) {
        moveTo(pose.x, pose.y, pose.h, posErrorTolerance, headingErrorTolerance, maxPower);
    }

    public void moveTo(double targetX, double targetY, double targetHeading, double posErrorTolerance, double headingErrorTolerance, double maxPower) {
        SparkFunOTOS.Pose2D pose = getPosition();

        double yaw = 0;
        double power = 0;
        double errorX = targetX - pose.x;
        double errorY = targetY - pose.y;

        double posError = Math.sqrt(errorX*errorX + errorY*errorY);
        double prevPosError = posError;
        double posErrorDiff = 0;

        double headingError = minimizeAngle(targetHeading - pose.h);
        double prevHeadingError = headingError;
        double headingErrorDiff = 0;

        int haltTimer = 0;
        while (opMode.opModeIsActive() && ((posError > posErrorTolerance) || (Math.abs(headingError) > headingErrorTolerance))) {
            prevPosError = posError;
            prevHeadingError = headingError;

            // recalculate position and error
            pose = getPosition();

            errorX = targetX - pose.x;
            errorY = targetY - pose.y;

            posError = Math.sqrt(errorX*errorX + errorY*errorY);
            headingError = minimizeAngle(targetHeading - pose.h);

            posErrorDiff = posError - prevPosError;
            headingErrorDiff = headingError - prevHeadingError;

            // remember: positive yaw means rotation COUNTERCLOCKWISE
            yaw = heading_p * headingError + heading_d * headingErrorDiff;
            yaw = constrainPower(minYaw, maxYaw, yaw);

            power = position_p * posError + position_d * posErrorDiff;
            power = constrainPower(minPower, maxPower, power);

            telemetryAll.addData("yaw", yaw);
            telemetryAll.addData("power", power);
            telemetryAll.addData("x", pose.x);
            telemetryAll.addData("y", pose.y);
            telemetryAll.update();

            // using error x and y, we know the exact direction we need to travel in the x and y directions
            moveFieldCentric(errorX, errorY, pose.h, power, yaw);

            // if we have stopped moving for some reason (likely because of hitting a wall)
            if (Math.abs(posErrorDiff) < 0.05) {
                // increase the halt timer
                haltTimer++;
            } else {
                // else reset the timer
                haltTimer = 0;
            }

            // if we have halted for over 120 cycles
            if (haltTimer > 120) {
                telemetryAll.addLine("exited from halt");
                telemetryAll.update();
                // exit from movement
                break;
            }
        }
        // stop the motors
        stopMotors();
    }
}