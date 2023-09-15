package frc.robot.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.maps.RobotMap;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrive;

public class TestAuto extends SequentialCommandGroup{

    // Create constructor for when this auto is called in RobotContainer
    // Constuctor must contain sybsystems used so that they can be referenced
    public TestAuto(SwerveDrive swerveDrive){
        TrajectoryConfig trajConfig = new TrajectoryConfig(RobotMap.MAX_SPEED_METERS_PER_SECOND, 0.5).setKinematics(RobotMap.DRIVE_KINEMATICS);

         Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), null, new Pose2d(1, 1, Rotation2d.fromDegrees(180)), trajConfig)
       
        super.addCommands(
            
        );
    }
}

// Inefficient, possibly condense everything into one autonomous class and use .getName()
// Might not work due to command-based nature of command selector