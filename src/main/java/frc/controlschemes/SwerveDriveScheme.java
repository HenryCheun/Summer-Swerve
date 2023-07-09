package frc.controlschemes;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

/**
 * Control scheme for swerve drive. Includes movement, the toggle between
 * field centric and robot centric, and a button to zero the gyro.
 */
public class SwerveDriveScheme implements ControlScheme{
    private static boolean fieldCentric = true;

    /**
     * Configures the basic driving as well as buttons.
     * @param swerveDrive The SwerveDrive object being configured.
     * @param port The controller port of the driving controller.
     */
    public static void configure(SwerveDrive swerveDrive, int port){
        SlewRateLimiter xRateLimiter = new SlewRateLimiter(RobotMap.DRIVE_RATE_LIMIT);
        SlewRateLimiter yRateLimiter = new SlewRateLimiter(RobotMap.DRIVE_RATE_LIMIT);
        SlewRateLimiter turnRateLimiter = new SlewRateLimiter(RobotMap.TURN_RATE_LIMIT);
        

        swerveDrive.setDefaultCommand(new RunCommand(() -> {
            //Set x, y, and turn speed based on joystick inputs
            double xSpeed = OI.axis(ControlMap.L_JOYSTICK_VERTICAL, port);
            double ySpeed = OI.axis(ControlMap.L_JOYSTICK_HORIZONTAL, port);
            double turnSpeed = OI.axis(ControlMap.R_JOYSTICK_HORIZONTAL, port);

            //Limits acceleration and speed
            //Possibly change the speed limiting to somewhere else (maybe a normalize function)
            xSpeed = xRateLimiter.calculate(xSpeed) * RobotMap.MAX_SPEED_METERS_PER_SECOND;
            ySpeed = yRateLimiter.calculate(ySpeed) * RobotMap.MAX_SPEED_METERS_PER_SECOND;
            turnSpeed = turnRateLimiter.calculate(turnSpeed) * RobotMap.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            //Constructs desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            if(fieldCentric){
                //Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveDrive.getRotation2d());
            } else {
                //Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            }

            //Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = RobotMap.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

            swerveDrive.setModuleStates(moduleStates);
        }, swerveDrive));
        configureButtons(swerveDrive, port);
    }

    /**
     * Configures buttons and their respective commands.
     * @param swerveDrive The SwerveDrive object being configured.
     * @param port The controller port of the driving controller.
     */
    private static void configureButtons(SwerveDrive swerveDrive, int port){
        new JoystickButton(controllers[port], ControlMap.B_BUTTON)
            .onTrue(new InstantCommand(() -> toggleFieldCentric()));
        new JoystickButton(controllers[port], ControlMap.A_BUTTON)
            .onTrue(new InstantCommand(() -> swerveDrive.zeroHeading()));
    }

    /**
     * Toggle field centric and robot centric driving.
     */
    private static void toggleFieldCentric(){
        fieldCentric = !fieldCentric;
    }
    
}
