package frc.maps;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import frc.robot.CCSparkMax;

/*
    RobotMap holds all the ports involved in the robot.
    This ranges from talon ports, all the way to the ports
    on the controllers. This also contains the polarity for the wheels
    and the various ports assoiated with sensors
    If you wish to create your own port, here is the syntax:
        public static final returnType name = value;
    Notes on creating ports:
        1. Ports must be integers or booleans
        2. they MUST be public static final;
        3. If the port is not plugged in, make int values -1, and boolean values false
*/
public interface RobotMap {
     //Swerve Module Constants

     // 1, 2, 3, 4, 5, 6, 7, ,8 

     // 4 drive
     // 4 turn - 1 

     public static final int FRONT_RIGHT_DRIVE = 6;
     public static final boolean FRONT_RIGHT_DRIVE_REVERSE = false;
     public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
     public static final int FRONT_RIGHT_TURN = 5;
     public static final boolean FRONT_RIGHT_TURN_REVERSE = false;
     public static final double FRONT_RIGHT_TURN_ENCODER = 1;

     public static final int FRONT_LEFT_DRIVE = 2;
     public static final boolean FRONT_LEFT_DRIVE_REVERSE = false;
     public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
     public static final int FRONT_LEFT_TURN = 1;
     public static final boolean FRONT_LEFT_TURN_REVERSE = false;
     public static final double FRONT_LEFT_TURN_ENCODER = 1;

     public static final int BACK_RIGHT_DRIVE = 7;
     public static final boolean BACK_RIGHT_DRIVE_REVERSE = false;
     public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
     public static final int BACK_RIGHT_TURN = 8;
     public static final boolean BACK_RIGHT_TURN_REVERSE = false;
     public static final double BACK_RIGHT_TURN_ENCODER = 1;

     public static final int BACK_LEFT_DRIVE = 3;
     public static final boolean BACK_LEFT_DRIVE_REVERSE = false;
     public static final double BACK_LEFT_DRIVE_ENCODER = 1;
     public static final int BACK_LEFT_TURN = 4;
     public static final boolean BACK_LEFT_TURN_REVERSE = false;
     public static final double BACK_LEFT_TURN_ENCODER = 1;
     
     //Absolute Encoders
     public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 3;
     public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 0;
     public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 2;
     public static final int BACK_LEFT_ABSOLUTE_ENCODER = 1;

     //Absolute Encoder Offsets (in radians)
     public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = Math.toRadians(-15);
     public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = Math.toRadians(-105);
     public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = Math.toRadians(-285);
     public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = Math.toRadians(-195);

    public static final double MAX_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

    //Rate Limiters (acceleration)
    public static final double DRIVE_RATE_LIMIT = MAX_SPEED_METERS_PER_SECOND / 4;
    public static final double TURN_RATE_LIMIT = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

    //Robot Dimensions (relative to wheel locations)
    //Since this robot is a square, no need for 2 values. In a non-square chassis, 2 values needed.
    public static final double WHEEL_BASE = Units.inchesToMeters(27);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, -WHEEL_BASE / 2),
        new Translation2d(WHEEL_BASE / 2, WHEEL_BASE / 2),
        new Translation2d(-WHEEL_BASE / 2, -WHEEL_BASE / 2),
        new Translation2d(-WHEEL_BASE / 2, WHEEL_BASE / 2)
    );
}
