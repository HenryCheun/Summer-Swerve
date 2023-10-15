package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class Balance extends CommandBase {

    private final SwerveDrive swerveDrive;
    private double dampen;
    private double lastAngle;
    private double baseSpeed;
    private SwerveModuleState[] moduleStates;

    public Balance(SwerveDrive swerveDrive) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        dampen = 1;
        lastAngle = 0;
        baseSpeed = 0.25;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    // TODO Might need to change roll to something else, and might also need to flip
    // signs.
    public void execute() {
        // double ang = -swerveDrive.gyro.getRoll();
        double ang = swerveDrive.getPitch();
        if (lastAngle * ang <= 0) {
            dampen *= 0.6;
            // Timer.delay(1);
        }
        ChassisSpeeds speeds;
        if (ang < -1) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-baseSpeed * dampen, 0, 0, swerveDrive.getRotation2d());
            moduleStates = RobotMap.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
            swerveDrive.setModuleStates(moduleStates);
        } else if (ang > 1) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(baseSpeed * dampen, 0, 0, swerveDrive.getRotation2d());
            moduleStates = RobotMap.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
            swerveDrive.setModuleStates(moduleStates);
        } else {
            swerveDrive.stopModules();
        }
        lastAngle = ang;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(lastAngle) < 1 && dampen < 1;
    }

}