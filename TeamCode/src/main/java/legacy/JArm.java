package legacy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JArm extends Thread {
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private Telemetry telemetryAll;

    private PIDController controller;
    public static double p = 0.0085, i = 0, d = 0.00010;
    public static double f = 0.005;
    public static double powerCap = 0.75;

    private final double volts_in_degree = 1.22;

    private DcMotor arm1;
    private DcMotor arm2;
    private AnalogInput pot;
    private TouchSensor armTopLimit;
    private TouchSensor armBottomLimit;

    private volatile double targetPot = 100;

    static double specimenPlace = 187.4;
    static double specimenGrab = 119.2;
    static double basketPlace = 290.8;
    static double parkingHeight = 200.0;

    public JArm(LinearOpMode initOpMode, HardwareMap initHardwareMap) {
        opMode = initOpMode;
        hardwareMap = initHardwareMap;

        controller = new PIDController(p,i,d);
        arm1 = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "armOther");
        pot = hardwareMap.get(AnalogInput.class, "pot");
        armTopLimit = hardwareMap.get(TouchSensor.class, "armTopLimit");
        armBottomLimit = hardwareMap.get(TouchSensor.class, "armBottomLimit");

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetryAll = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryAll.addData("Status", "Initialized");
        telemetryAll.update();
    }

    public void setTarget(double newPot) {
        targetPot = newPot;
    }

    public double getTarget() {
        return targetPot;
    }

    public void run() {
        boolean pidMode = true;
        boolean holdMode = true;
        boolean startOfHold = true;
        int holdEncoder = 0;
        double power = 0;

        while (opMode.opModeIsActive()) {
            double currentPot = pot.getVoltage()*100;
            int currentEnc = arm1.getCurrentPosition();

            if (Math.abs(targetPot - currentPot) < 3) {
                pidMode = false;
                holdMode = true;
                startOfHold = true;
            } else {
                pidMode = true;
                holdMode = false;
            }

            if (holdMode) {
                if (startOfHold) {
                    holdEncoder = currentEnc;
                    startOfHold = false;
                }

                arm1.setTargetPosition(holdEncoder);
                arm2.setTargetPosition(holdEncoder);
                power = 0.5;
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (pidMode) {
                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                controller.setPID(p, i, d);
                double pid = controller.calculate(currentPot, targetPot);
                double ff = Math.cos(Math.toRadians(targetPot / volts_in_degree)) * f;

                power = pid + ff;

                // Scale max power down for testing
                if(Math.abs(power) > powerCap)
                {
                    power = Math.signum(power) * powerCap;
                }

                // Check for limit switches
                if (power < 0 && armBottomLimit.isPressed())
                {
                    power = 0;
                }
                else if (power > 0 && armTopLimit.isPressed())
                {
                    power = 0;
                }
            }

            arm1.setPower(power);
            arm2.setPower(power);

            telemetryAll.addData("pot", currentPot);
            telemetryAll.addData("power", power);
            telemetryAll.addData("target", targetPot);
            telemetryAll.addData("encpos", currentEnc);
            telemetryAll.addData("enctarget", holdEncoder);
            telemetryAll.addData("hold mode: ", holdMode);
            telemetryAll.addData("pid mode: ", pidMode);
            telemetryAll.update();
        }
    }
}
