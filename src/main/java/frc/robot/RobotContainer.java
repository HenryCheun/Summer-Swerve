package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.controlschemes.SwerveDriveScheme;
import frc.controlschemes.Testing;
import frc.diagnostics.CommandSelector;
import frc.diagnostics.StringSelector;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    SwerveDrive swerveDrive = new SwerveDrive();

    /** Event map for path planner */
    public static HashMap<String, Command> eventMap = new HashMap<>();

    public RobotContainer() {
        SwerveDriveScheme.configure(swerveDrive, 0);
        // Testing.configure(swerveDrive, 0);
    }


    // Add all autonomous paths here.
    StringSelector selector = new StringSelector(
        "Autonomous Path",
        "Test Auto"
    );

    public Command getAutoCommand() {
        return new Autonomous(selector.value(), swerveDrive);
    }

    
}