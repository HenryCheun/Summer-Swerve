package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.controlschemes.SwerveDriveScheme;
import frc.controlschemes.Testing;
import frc.diagnostics.CommandRunner;
import frc.diagnostics.CommandSelector;
import frc.diagnostics.StringSelector;
import frc.maps.RobotMap;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    SwerveDrive swerveDrive = new SwerveDrive();

    /** Event map for path planner */
    public static HashMap<String, Command> eventMap = new HashMap<>();

    SendableChooser<Command> autoCommands = new SendableChooser<Command>();
    private final String[] paths = {"First Test",
            "Straight",
            "New Path",
            "curve",
            "Straight Then Left",
            "Rotate Right" };
    
    public RobotContainer() {
        SwerveDriveScheme.configure(swerveDrive, 0);
        for (String pathName : paths) {
            autoCommands.addOption(pathName, createPath(pathName));
        }
        // Testing.configure(swerveDrive, 0);
    }

    // Add all autonomous paths here.
    StringSelector selector = new StringSelector(
            "Autonomous Path",
            "First Test",
            "Straight",
            "New Path",
            "curve",
            "Straight Then Left",
            "Rotate Right");

    CommandRunner runTutorialPath = new CommandRunner("Config", "Tutorial path", followPathPlanner());

    public Command getAutoCommand() {
        // return new Autonomous(selector.value(), swerveDrive);
        return followPathPlanner();
    }

    /**
     * Functionally the same as the Autonomous class method, just less messy.
     */
    public Command followPathPlanner() {
        String pathName = selector.value();
        PathPlannerTrajectory traj = PathPlanner.loadPath(pathName,
                new PathConstraints(RobotMap.MAX_SPEED_METERS_PER_SECOND - 1.5,
                        RobotMap.DRIVE_RATE_LIMIT - .3));

        return Commands.sequence(
                Commands.waitSeconds(1),
                Commands.runOnce(swerveDrive::resetOdometry, swerveDrive),
                swerveDrive.followPath(traj),
                Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    }

    


/**
 * 
 * @return Command to follow the tutorial path
 */
public Command followTutorialPath() {
    
        TrajectoryConfig config = new TrajectoryConfig(RobotMap.MAX_SPEED_METERS_PER_SECOND, RobotMap.DRIVE_RATE_LIMIT)
                .setKinematics(RobotMap.DRIVE_KINEMATICS);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), null,
                new Pose2d(1, 1, new Rotation2d(0)), config);

                PIDController xPID = new PIDController(.5, .15, 0);
                PIDController yPID = new PIDController(.5, .15, 0);

                return new InstantCommand();

    }
}


// SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

//     public RobotContainer() {
//         ...
//         m_autoSelector.setDefaultOption("default auto", followPath("Default Path"));
//         m_autoSelector.addOption("other auto", followPath("Other Path"));
//     }
//     public Command getAutonomousCommand() {
//         return m_autoSelector.getSelected();
//     }
//     public Command followPath(String pathName) {
//          return followPath(PathPlanner.loadPath(pathName, constraints));
//     }
//     public Command followPath(PathPlannerTrajectory traj){
//              //as you have it now
//     }


// new CommandSelector("Main Tab", "Auto Selector",
// followPath("DefaultAuto").withName("Default Auto"),
// followPath("OtherAuto").withName("Other Auto")
// );



// SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

//     public RobotContainer() {
//         ...
//         m_autoSelector.setDefaultOption("default auto", followPath("Default Path"));
//         m_autoSelector.addOption("other auto", followPath("Other Path"));

//         Shuffleboard.getTab("Config")
//             .add("AutoChooser", m_autoSelector)
//             .withWidget("ComboBox Chooser")
//             .withPosition(1, 1)
//             .withSize(1, 1);
//     }
//     public Command getAutonomousCommand() {
//         return m_autoSelector.getSelected();
//     }
//     public Command followPath(String pathName) {
//          return followPath(PathPlanner.loadPath(pathName, constraints));
//     }
//     public Command followPath(PathPlannerTrajectory traj){
//              //as you have it now
//     }
