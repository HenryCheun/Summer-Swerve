package frc.controlschemes;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.diagnostics.BooleanSwitch;
import frc.diagnostics.CommandSelector;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

/**
 * Control scheme for swerve drive. Includes movement, the toggle between
 * field centric and robot centric, and a button to zero the gyro.
 */
public class SwerveDriveScheme implements ControlScheme {
    private static boolean fieldCentric = true;
    private static BooleanSupplier fieldCentricSupplier = () -> {
        return fieldCentric;
    };

    /**
     * Configures the basic driving as well as buttons.
     * @param swerveDrive The SwerveDrive object being configured.
     * @param port The controller port of the driving controller.
     */
    public static void configure(SwerveDrive swerveDrive, int port){
        Shuffleboard.getTab("Diagnostics").getLayout("Swerve", "List").add("isCentric",fieldCentric).withWidget(BuiltInWidgets.kBooleanBox);
       Shuffleboard.getTab("Diagnostics").addBoolean("Field Centric", fieldCentricSupplier).withWidget(BuiltInWidgets.kToggleSwitch);

        SlewRateLimiter xRateLimiter = new SlewRateLimiter(RobotMap.DRIVE_RATE_LIMIT);
        SlewRateLimiter yRateLimiter = new SlewRateLimiter(RobotMap.DRIVE_RATE_LIMIT);
        SlewRateLimiter turnRateLimiter = new SlewRateLimiter(RobotMap.TURN_RATE_LIMIT);
        
        // InstantCommand com1 = new InstantCommand(() -> swerveDrive.printFrontRight(), swerveDrive);
        // com1.setName("Front Right");
        // InstantCommand com2 = new InstantCommand(() -> swerveDrive.printFrontLeft(), swerveDrive);
        // com2.setName("Front Left");
        // InstantCommand com3 = new InstantCommand(() -> swerveDrive.printBackRight(), swerveDrive);
        // com3.setName("Back Right");
        // InstantCommand com4 = new InstantCommand(() -> swerveDrive.printBackLeft(), swerveDrive);
        // com4.setName("Back Left");
        // CommandSelector commands = new CommandSelector(
        //         "Motor Print",
        //         com1,
        //         com2,
        //         com3,
        //         com4);
        

        swerveDrive.setDefaultCommand(new RunCommand(() -> {
            // swerveDrive.periodic();

            //Set x, y, and turn speed based on joystick inputs
            double xSpeed = -OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL);
            double ySpeed = -OI.axis(port, ControlMap.L_JOYSTICK_HORIZONTAL);
            double turnSpeed = -OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL);

            //Limits acceleration and speed
            //Possibly change the speed limiting to somewhere else (maybe a normalize function)
            xSpeed = xRateLimiter.calculate(xSpeed);
            ySpeed = yRateLimiter.calculate(ySpeed);
            turnSpeed = turnRateLimiter.calculate(turnSpeed);

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
            

            // commands.value().schedule();
        }, swerveDrive).withName("Swerve Controller Command"));
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
