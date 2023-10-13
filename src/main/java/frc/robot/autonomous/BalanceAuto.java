package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class BalanceAuto extends SequentialCommandGroup {
    public BalanceAuto(SwerveDrive swerve) {
        super.addCommands(
            new Balance(swerve)
        );
        super.addCommands(
            new InstantCommand(() -> swerve.setOdometry(trajectory.getInitialPose()), swerve),
        );
    }
}
