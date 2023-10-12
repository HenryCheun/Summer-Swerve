import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class Balance extends CommandBase {

    private final SwerveDrive swerveDrive;
    private double dampen;
    private double lastAngle;
    private double baseSpeed;

    public Balance(SwerveDrive swerveDrive) {
        // Use addRequirements() here to declare subsystem dependencies.
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
    //TODO Might need to change roll to something else, and might also need to flip signs. 
    public void execute() {
        // double ang = -swerveDrive.gyro.getRoll();
        double ang = swerveDrive.getRoll(); 
        if (lastAngle * ang <= 0) {
            dampen *= 0.6;
            // Timer.delay(1);
        }
        if (ang < -4) {
            swerveDrive.setModuleStates(
                    new SwerveModuleState[] { new SwerveModuleState(baseSpeed * dampen, new Rotation2d(0)),
                            new SwerveModuleState(baseSpeed * dampen, new Rotation2d(0)),
                            new SwerveModuleState(baseSpeed * dampen, new Rotation2d(0)),
                            new SwerveModuleState(baseSpeed * dampen, new Rotation2d(0)) });
        } else if (ang > 4)
            swerveDrive.setModuleStates(
                    new SwerveModuleState[] { new SwerveModuleState(-baseSpeed * dampen, new Rotation2d(0)),
                            new SwerveModuleState(-baseSpeed * dampen, new Rotation2d(0)),
                            new SwerveModuleState(-baseSpeed * dampen, new Rotation2d(0)),
                            new SwerveModuleState(-baseSpeed * dampen, new Rotation2d(0)) });
        else
            swerveDrive.stopModules();
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