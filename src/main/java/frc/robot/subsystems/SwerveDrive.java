package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;
import frc.robot.RobotContainer;

/**
 * Class for controlling a swerve drive chassis. Consists of 4 SwerveModules and a gyro.
 */
public class SwerveDrive extends SubsystemBase {
    //Initializing swerve modules. Must include full CCSparkMax object declarations.
    private final SwerveModule frontRight = new SwerveModule(
        new CCSparkMax(
            "Front Right Drive",
            "frd",
            RobotMap.FRONT_RIGHT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kCoast,
            RobotMap.FRONT_RIGHT_DRIVE_REVERSE,
            RobotMap.UNIVERSAL_DRIVE_ENCODER_FACTOR),
        new CCSparkMax(
            "Front Right Turn",
            "frt",
            RobotMap.FRONT_RIGHT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            RobotMap.FRONT_RIGHT_TURN_REVERSE,
            RobotMap.UNIVERSAL_TURN_ENCODER_FACTOR),
        RobotMap.FRONT_RIGHT_ABSOLUTE_ENCODER,
        RobotMap.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET,
        "Front Right"
    );

    private final SwerveModule frontLeft = new SwerveModule(
        new CCSparkMax(
            "Front Left Drive",
            "fld",
            RobotMap.FRONT_LEFT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kCoast,
            RobotMap.FRONT_LEFT_DRIVE_REVERSE,
            RobotMap.UNIVERSAL_DRIVE_ENCODER_FACTOR),
        new CCSparkMax(
            "Front Left Turn",
            "flt",
            RobotMap.FRONT_LEFT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            RobotMap.FRONT_LEFT_TURN_REVERSE,
            RobotMap.UNIVERSAL_TURN_ENCODER_FACTOR),
        RobotMap.FRONT_LEFT_ABSOLUTE_ENCODER,
        RobotMap.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET,
        "Front Left"
    );

    private final SwerveModule backRight = new SwerveModule(
        new CCSparkMax(
            "Back Right Drive",
            "brd",
            RobotMap.BACK_RIGHT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kCoast,
            RobotMap.BACK_RIGHT_DRIVE_REVERSE,
            RobotMap.UNIVERSAL_DRIVE_ENCODER_FACTOR),
        new CCSparkMax(
            "Back Right Turn",
            "brt",
            RobotMap.BACK_RIGHT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            RobotMap.BACK_RIGHT_TURN_REVERSE,
            RobotMap.UNIVERSAL_TURN_ENCODER_FACTOR),
        RobotMap.BACK_RIGHT_ABSOLUTE_ENCODER,
        RobotMap.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET,
        "Back Right"
    );

    private final SwerveModule backLeft = new SwerveModule(
        new CCSparkMax(
            "Back Left Drive",
            "bld",
            RobotMap.BACK_LEFT_DRIVE,
            MotorType.kBrushless,
            IdleMode.kCoast,
            RobotMap.BACK_LEFT_DRIVE_REVERSE,
            RobotMap.UNIVERSAL_DRIVE_ENCODER_FACTOR),
        new CCSparkMax(
            "Back Left Turn",
            "blt",
            RobotMap.BACK_LEFT_TURN,
            MotorType.kBrushless,
            IdleMode.kBrake,
            RobotMap.BACK_LEFT_TURN_REVERSE,
            RobotMap.UNIVERSAL_TURN_ENCODER_FACTOR),
        RobotMap.BACK_LEFT_ABSOLUTE_ENCODER,
        RobotMap.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET,
        "Back Left"
    );

    //Initialize gyro
    private AHRS gyro = new AHRS(SPI.Port.kMXP);


    

    /** Module positions used for odometry */
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    
    /**
     * Creates a new SwerveDrive object. Delays 1 second before setting gyro to 0 to account for gyro calibration time.
     */
    public SwerveDrive(){
        swerveModulePositions[0] = new SwerveModulePosition(0, new Rotation2d(frontRight.getAbsoluteEncoderRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(0, new Rotation2d(frontLeft.getAbsoluteEncoderRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(0, new Rotation2d(backRight.getAbsoluteEncoderRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(0, new Rotation2d(backLeft.getAbsoluteEncoderRadians()));

        odometer = new SwerveDriveOdometry(RobotMap.DRIVE_KINEMATICS, new Rotation2d(0), swerveModulePositions);

        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e){
            }
        }).start();
    }

    /**
     * Resets chassis gyro to 0. For all gyro purposes, "heading"  refers to the facing direction of the gyro.
     */
    public void zeroHeading(){
        gyro.reset();
    }

    /**
     * Method to get the facing direction of the gyro. 
     * @return The facing direction of the gyro, between -360 and 360 degrees.
     */
    public double getHeading(){
        return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    /**
     * Gets the Rotation2d value of the facing direction of the robot.
     * @return The facing direction of the robot in Rotation2d format.
     */
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Debug function.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // System.out.println("Robot Heading: " + getHeading());
        updateOdometer();
    }

    /**
     * Sets all 4 modules' drive and turn speeds to 0.
     */
    public void stopModules(){
        // frontRight.stop();
        // frontLeft.stop();
        // backRight.stop();
        // backLeft.stop();
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(frontRight.getAbsoluteEncoderRadians())),
            new SwerveModuleState(0, new Rotation2d(frontLeft.getAbsoluteEncoderRadians())),
            new SwerveModuleState(0, new Rotation2d(backRight.getAbsoluteEncoderRadians())),
            new SwerveModuleState(0, new Rotation2d(backLeft.getAbsoluteEncoderRadians()))
        };
        setModuleStates(states);
    }

    /**
     * Sets all 4 modules' drive and turn speeds with the SwerveModuleState format.
     * @param desiredStates The array of the states that each module will be set to in the SwerveModuleState format.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RobotMap.MAX_SPEED_METERS_PER_SECOND);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    /**
     * Gets the position of the robot in Pose2d format. Uses odometer reading.
     * Includes the x, y, and theta values of the robot. 
     * @return The Pose2d of the robot.
     */
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

    /**
     * Resets the odometer readings using the gyro, SwerveModulePositions (defined in constructor), and Pose2d.
     */
    public void resetOdometry(){
        odometer.resetPosition(getRotation2d(), swerveModulePositions, getPose());
    }

    public void setOdometry(Pose2d pos){
        odometer.resetPosition(getRotation2d(), swerveModulePositions, pos);
    }

    public void updateModulePositions(){
        swerveModulePositions[0] = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition()));
        swerveModulePositions[1] = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition()));
        swerveModulePositions[2] = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition()));
        swerveModulePositions[3] = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition()));
    }

    public void updateOdometer(){
        updateModulePositions();
        odometer.update(getRotation2d(), swerveModulePositions);
    }
    // Autonomous
    // Odometer used to get Pose2d of the robot.
    SwerveDriveOdometry odometer;

    // xPID and yPID should have the same values.
    PIDController xPID = new PIDController(.5, .15, 0);
    PIDController yPID = new PIDController(.5, .15, 0);
    // Possibly research profiled PID
    PIDController turnPID = new PIDController(.25, 0, 0);

    // PPHolonomicDriveController holonomicDriveController = new PPHolonomicDriveController(xPID, yPID, turnPID);

    /**
     * Follows a PathPlanner path. Referenced in autonomous classes.
     * @param traj The PathPlannerTrajectory to be followed.
     * @param firstPath Whether or not this is the first path being followed in auto. If so, resets the gyro before starting.
     * @return A command that follows a path.
     */
    public Command followPath(PathPlannerTrajectory traj){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                setOdometry(traj.getInitialHolonomicPose());
            }),
            new ParallelCommandGroup(
                new FollowPathWithEvents(
                    //Path following command
                    new PPSwerveControllerCommand(traj, this::getPose, RobotMap.DRIVE_KINEMATICS_AUTONOMOUS, xPID, yPID, turnPID, this::setModuleStates, true, this),
                    traj.getMarkers(),
                    RobotContainer.eventMap)
                // new RunCommand(() -> System.out.println(getPose()))
            )
        );
    }

    public void test(double driveSpeed, double turnSpeed){
        backLeft.driveAndTurn(driveSpeed, turnSpeed);
        backLeft.printEncoders();
    }

    public void resetAbsoluteEncoders(){
        frontRight.resetAbsoluteEncoder();
        frontLeft.resetAbsoluteEncoder();
        backRight.resetAbsoluteEncoder();
        backLeft.resetAbsoluteEncoder();
    }

    public void printAbsoluteEncoders(){
        frontRight.printAbsoluteEncoder();
        frontLeft.printAbsoluteEncoder();
        backRight.printAbsoluteEncoder();
        backLeft.printAbsoluteEncoder();
    }

    public void resetEncoders(){
        frontRight.resetEncoders();
        frontLeft.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
    }

    public void printFrontRight(){
        frontRight.printEncoders();
        frontRight.printAbsoluteEncoder();
    }

    public void printFrontLeft(){
        frontLeft.printEncoders();
        frontLeft.printAbsoluteEncoder();
    }

    public void printBackRight(){
        backRight.printEncoders();
        backRight.printAbsoluteEncoder();
    }

    public void printBackLeft(){
        backLeft.printEncoders();
        backLeft.printAbsoluteEncoder();
    }

    public void printPos2d(){
        System.out.println(odometer.getPoseMeters());
    }
}
