package frc.robot.constants;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // TODO This is a constant being managed like a static rewriteable variable
    public static RobotConfig kRobotConfig;

    public static final PPHolonomicDriveController kPPDriveController = new PPHolonomicDriveController(
        new PIDConstants(kPXController, 0, 0),
        new PIDConstants(kPYController, 0, 0)
    );

    static {
        try {
            kRobotConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            System.err.println("FAILED TO READ ROBOTCONFIG, WAS THE CONFIG SET UP IN PATHPLANNER?");
            e.printStackTrace();
        }
    }
}
