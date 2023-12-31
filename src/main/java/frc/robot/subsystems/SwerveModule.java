package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;

/**
 * Class for controlling a swerve module. Each module has 2 motors, one for driving and one for turning, as well as an absolute encoder.
 * 
 * <p>Swerve modules are set to different positions and vectors in the SwerveModuleState format.
 * SwerveModuleState takes in a drive speed in meters per second and an angle in radians in the
 * format of Rotation2d.
 */
public class SwerveModule extends SubsystemBase{
    private CCSparkMax driveMotor;
    private CCSparkMax turnMotor;

    private PIDController turningPIDController;

    private AnalogEncoder absoluteEncoder;
    private double absoluteEncoderOffset;

    private String name;

    //adjust absoluteEncoderChannel to possibly be absoluteEncoderAnalogInput
    /**
     * Creates a SwerveModule object with a defined drive motor, turn motor, and absolute encoder.
     * @param driveMotor The drive motor in CCSparkMax format.
     * @param turnMotor The turn motor in CCSparkMax format.
     * @param absoluteEncoderChannel The port of the absolute encoder.
     * @param absoluteEncoderOffset The offset of the absolute encoder in radians.
     */
    public SwerveModule(CCSparkMax driveMotor, CCSparkMax turnMotor, int absoluteEncoderChannel, double absoluteEncoderOffset, String name){
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.absoluteEncoder = new AnalogEncoder(absoluteEncoderChannel);
        // this.absoluteEncoder.setDistancePerRotation(2 * Math.PI);
        //add encoder offset

        turningPIDController = new PIDController(.5, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.name = name;
        resetEncoders();
    }

    /**
     * Gets the encoder value of the drive motor in meters.
     * @return The encoder value of the drive motor.
     */
    public double getDrivePosition(){
        return driveMotor.getPosition();
    } 

    /**
     * Gets the encoder value of the turn motor.
     * @return The encoder value of the turn motor.
     */
    public double getTurnPosition(){
        return turnMotor.getPosition();
    } 

    /**
     * Gets the speed of the drive motor.
     * @return The speed of the drive motor between -1 and 1.
     */
    public double getDriveVelocity(){
        return driveMotor.getSpeed();
    } 

    /**
     * Gets the speed of the turn motor.
     * @return The speed of the turn motor between -1 and 1.
     */
    public double getTurnVelocity(){
        return turnMotor.getSpeed();
    }

    /**
     * Gets the reading of the absolute encoder.
     * @return The value of the absolute encoder in radians.
     */
    public double getAbsoluteEncoderRadians(){
        return absoluteEncoder.get() * 2 * Math.PI;
    }
    
    /**
     * Resets the drive and turn motor encoders. The drive motor is set to
     * 0 while the turn motor is set to the value of the absolute encoder.
     */
    public void resetEncoders(){
        driveMotor.reset();
        // turnMotor.setPosition(getAbsoluteEncoderRadians());
        turnMotor.reset();
    }

    /**
     * Gets the state of the module.
     * @return The state of the swerve module in SwerveModuleState format.
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    /**
     * Sets the state of the module.
     * @param state The state to set the swerve module to in SwerveModuleState format.
     */
    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) <= .01){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        //integrate max speed here
        driveMotor.set(state.speedMetersPerSecond);
        turnMotor.set(turningPIDController.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    /**
     * Sets the speed of the drive and turn motors to 0.
     */
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Method for testing purposes
     * @param driveSpeed Speed of the drive motor.
     * @param turnSpeed Speed of the turn motor.
     */
    public void driveAndTurn(double driveSpeed, double turnSpeed){
        driveMotor.set(driveSpeed);
        turnMotor.set(turnSpeed);
    }

    public void printEncoders(){
        System.out.println(name + "\nDrive Encoder: " + driveMotor.getPosition() + "\nTurn Encoder: " + turnMotor.getPosition() + "\n");
    }

    public void resetAbsoluteEncoder(){
        absoluteEncoder.reset();
    }

    public void printAbsoluteEncoder(){
        System.out.println(name + ": " + absoluteEncoder.getDistance());
    }

    public String getName(){
        return name;
    }

    public void setName(String name){
        this.name = name;
    }
}
