/*
Copyright 2025 FIRST Tech Challenge Team 15297

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package legacy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp
@Config
@Disabled
public class DeepArmPidPotTuner extends LinearOpMode {

    private PIDController controller;
    public static double p = 0.008, i = 0, d = 0.0001;
    public static double f = 0.005;
    public static double powerCap = 0.75;

    public static double target = 150;
    private final double volts_in_degree = 1.22;

    private AnalogInput potentiometer;
    private DcMotorEx arm ;
    private DcMotorEx armOther;
    private TouchSensor armTopLimit;
    private TouchSensor armBottomLimit;

    private Telemetry telemetryAll;

    private boolean dUp = false;
    private boolean dDown = false;

    private int correctCount = 0;
    public static boolean powerMode = false;
    public static boolean pidMode = false;
    public static boolean holdMode = true;
    private boolean startOfHold = true;

    private double controllerPower = 0;
    private double power;
    private int holdEncoder;

    @Override
    public void runOpMode() {
        controller = new PIDController(p,i,d);

        potentiometer = hardwareMap.analogInput.get("pot");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        armOther = hardwareMap.get(DcMotorEx.class, "armOther");
        armTopLimit = hardwareMap.get(TouchSensor.class, "armTopLimit");
        armBottomLimit = hardwareMap.get(TouchSensor.class, "armBottomLimit");

        telemetryAll = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryAll.setNumDecimalPlaces(0, 3);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOther.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armOther.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armOther.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetryAll.addData("Status", "Initialized");
        telemetryAll.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetryAll.addData("Status", "Running");
            // telemetryAll.update();

            if (gamepad2.a)
            {
                target = 119.2;
                pidMode = true;
            }
            if (gamepad2.y)
            {
                target = 187.4;
                pidMode = true;
            }
            if (gamepad2.b)
            {
                target = 290.8;
                pidMode = true;
            }
            if (gamepad2.x)
            {
                target = 290;
                pidMode = true;
            }

            if (gamepad2.dpad_up)
            {
                if (!dUp)
                {
                    target += 2;
                    dUp = true;
                }
            }
            else
            {
                dUp = false;
            }
            if (gamepad2.dpad_down)
            {
                if (!dDown)
                {
                    target -= 2;
                    dDown = true;
                }
            }
            else
            {
                dDown = false;
            }

            // Check for valid target range 900mV to 3.0V (multiplied by 1000)
            if (target < 50)
            {
                target = 50;
            }
            if (target > 300)
            {
                target = 300;
            }

            double armPos = potentiometer.getVoltage() * 100;
            controllerPower = -gamepad2.left_stick_y;

            if (pidMode && power < 0.01 && Math.abs(target - armPos) < 3) {
                pidMode = false;
                holdMode = true;
                startOfHold = true;
            } else {
                holdMode = false;
            }

            if (Math.abs(controllerPower) > 0.1) {
                pidMode = false;
                holdMode = false;
                powerMode = true;
            } else {
                if (!pidMode) {
                    holdMode = true;
                }
                powerMode = false;
            }

            if (powerMode) {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armOther.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                power = controllerPower;
                startOfHold = true;
            } else if (holdMode) {
                if (startOfHold) {
                    holdEncoder = arm.getCurrentPosition();
                    startOfHold = false;
                }

                arm.setTargetPosition(holdEncoder);
                armOther.setTargetPosition(holdEncoder);
                power = 0.6;
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armOther.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (pidMode) {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armOther.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                controller.setPID(p, i, d);
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / volts_in_degree)) * f;

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

            arm.setPower(power);
            armOther.setPower(power);

            telemetryAll.addData("pot", armPos);
            telemetryAll.addData("power", power);
            telemetryAll.addData("target", target);
            telemetryAll.addData("encpos", arm.getCurrentPosition());
            telemetryAll.addData("enctarget", holdEncoder);
            telemetryAll.addData("hold mode: ", holdMode);
            telemetryAll.addData("pid mode: ", pidMode);
            telemetryAll.addData("power mode: ", powerMode);
            telemetryAll.update();
        }
    }
}
