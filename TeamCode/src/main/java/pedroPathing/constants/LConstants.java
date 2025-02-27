package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "otos";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(-1.87, 0.0, Math.toRadians(270.0));
        // Minimum and Maximum scalar values for the linear and angular scalars (from OTOS lib code)
        // MIN_SCALAR = 0.872;
        // MAX_SCALAR = 1.127;
        OTOSConstants.linearScalar = 1.0118583;
        OTOSConstants.angularScalar = 1.0;
    }
}




