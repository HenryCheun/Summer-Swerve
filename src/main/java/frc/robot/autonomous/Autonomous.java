package frc.robot.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class Autonomous extends SequentialCommandGroup{

    // Create constructor for when this auto is called in RobotContainer
    // Constuctor must contain sybsystems used so that they can be referenced

    /**
     * Create a new autonomous instance. Must pass in the name of the .path file and any subsystems used.
     * @param path The name of the .path file used. Must omit the ".path". All .path files are
     * under the deploy folder and are uploaded directly to the roboRIO.
     * @param swerveDrive The SwerveDrive subsystem used.
     */
    public Autonomous(String path, SwerveDrive swerveDrive){

        // This adds the commands to the class that this class extends, SequentialCommandGroup
        // So, when you call this auto in RobotContainer, it will run these commands
        // NOTE: ALL LINES HERE MUST BE COMMANDS
        super.addCommands(
            new InstantCommand(() -> {
                swerveDrive.resetOdometry();
            }),
            swerveDrive.followPath(PathPlanner.loadPath(path, new PathConstraints(RobotMap.MAX_SPEED_METERS_PER_SECOND, RobotMap.DRIVE_RATE_LIMIT)))
        );
    }
}

// Inefficient, possibly condense everything into one autonomous class and use .getName()
// Might not work due to command-based nature of command selector