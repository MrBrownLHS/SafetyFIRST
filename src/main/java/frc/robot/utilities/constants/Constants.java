package frc.robot.utilities.constants;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.ModuleConfiguration;
import frc.robot.utilities.SwerveModuleGearing;

public class Constants {
    public static final int gyroID = 0;

    public static final class CollectorArmConstants {
        // CAN IDs
        public static final int LIFT_MOTOR_1_ID = 13;
        public static final int LIFT_ENCODER_ID = 14;
        public static final int LIFT_MOTOR_2_ID = 15;
        public static final int PIVOT_MOTOR_1_ID = 16;
        public static final int PIVOT_ENCODER_ID = 17;
        public static final int PIVOT_MOTOR_2_ID = 18;
        public static final int TOP_INTAKE_MOTOR_ID = 19;
        public static final int BOTTOM_INTAKE_MOTOR_ID = 20;
        public static final int ARTICULATE_MOTOR_ID = 21;
        
        // PID Constants - NEEDS TUNING
        public static final double LIFT_kP = 0.01;
        public static final double LIFT_kI = 0.00;
        public static final double LIFT_kD = 0.01;
        
        public static final double PIVOT_kP = 0.01;
        public static final double PIVOT_kI = 0.00;
        public static final double PIVOT_kD = 0.01;
       
        // Feedforward/Trapazoidal Constants - NEEDS TUNING
        public static final double LIFT_kS = 0.0;
        public static final double LIFT_kG = 0.0;
        public static final double LIFT_kV = 0.0;
        public static final double LIFT_kA = 0.0;

        public static final double PIVOT_kS = 0.0;
        public static final double PIVOT_kG = 0.0;
        public static final double PIVOT_kV = 0.0;
        public static final double PIVOT_kA = 0.0;

        public static final double LIFT_MAX_VELOCITY = 3.0; // Max velocity of the lift - NEEDS TUNING
        public static final double LIFT_MAX_ACCELERATION = 1.0; // Max acceleration of the lift - NEEDS TUNING
        public static final double PIVOT_MAX_VELOCITY = 5.0; // Max velocity of the pivot - NEEDS TUNING
        public static final double PIVOT_MAX_ACCELERATION = 1.0; // Max acceleration of the pivot - NEEDS TUNING
        public static final double PIVOT_MIN_ANGLE = 30; // Min angle of the pivot - NEEDS TUNING
        public static final double PIVOT_MAX_ANGLE = 180; // Max angle of the pivot - NEEDS TUNING
        public static final double LIFT_MIN_HEIGHT = 0; // Min height of the lift - NEEDS TUNING
        public static final double LIFT_MAX_HEIGHT = 50; // Max height of the lift - NEEDS TUNING



        // Lift Encoder Postions - NEEDS TUNING
        public static final double LIFT_TOLERANCE = 2.0; //Tolerance for PID - NEEDS TUNING
        public static final double PIVOT_TOLERANCE = 2.0; //Tolerance for PID - NEEDS TUNING
         
        public static final double ENCODER_TO_DEGREES = 360.0 / 4096.0; 
        public static final double COG_DIAMETER_INCHES = 2.0; // Diameter of the center of gravity pulley in inches - NEEDS TUNING
        public static final double LIFT_GEAR_RATIO = 1.0; // Gear ratio of the lift mechanism - NEEDS TUNING
        public static final double ENCODER_TO_INCHES = (Math.PI * COG_DIAMETER_INCHES) / 360.0 / LIFT_GEAR_RATIO; 
        public static final double DEADBAND = 0.05;
        

        //Intake Motor Speeds - NEEDS TUNING
        public static final double INTAKE_SPEED = 0.25; //Intake Speed - NEEDS TUNING
        public static final double OUTTAKE_SPEED = 0.25; //Outtake Speed - NEEDS TUNING, may need to be negative
        public static final double ARTICULATE_SPEED = 0.25; //Articulate Speed - NEEDS TUNING
        public static final double ARTICULATE_DEADBAND = 0.05; //Articulate Deadband - NEEDS TUNING
        public static final double ARTICULATE_RATE_LIMIT = 0.5; //Articulate Rate Limit - NEEDS TUNING

        public static final int YEET_SPEED = 1;
        
        //Motor Configuration
        public static final double VOLTAGE_COMPENSATION = 12.0;
        public static final int CURRENT_LIMIT_NEO = 25;
        public static final int CURRENT_THRESHOLD_NEO = 40;
        public static final double CURRENT_THRESHOLD_TIME_NEO = 0.1;
        public static final int MAX_CURRENT_LIMIT_NEO = 60;
        public static final boolean ENABLE_CURRENT_LIMIT_NEO = true;

        public static final int CURRENT_LIMIT_550 = 20;
        public static final int CURRENT_THRESHOLD_550 = 40;
        public static final double CURRENT_THRESHOLD_TIME_550 = 10;
        public static final int MAX_CURRENT_LIMIT_550 = 60;
        public static final boolean ENABLE_CURRENT_LIMIT_550 = true;


    };

    public static final class CageClimberConstants {
        public static final int WINCH_MOTOR_1_ID = 22;
        public static final int WINCH_MOTOR_2_ID = 23;

        
    }

    public static final class ModuleConstants {
        /* Module Voltage Compensation */
        public static final double voltageCompensation = 12.0;

        /* Module Current Limiting */
        public static final int steeringCurrentLimit = 25;
        public static final int steeringCurrentThreshold = 40;
        public static final double steeringCurrentThresholdTime = 0.1;
        public static final boolean steeringEnableCurrentLimit = true;

        public static final int driveCurrentLimitNEO = 40;
        public static final int driveCurrentLimitCTRE = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        public static final int maximumCurrentLimit = 60;

        /* PID Profiling. Used to correct position errors */
        public static final double steeringkP = 0.01; // Proportional: If there is error, move the motor proportional to the error
        public static final double steeringkI = 0.00; // Integral: Creates a sum of the error over time. Increases until at set position
        public static final double steeringkD = 0.00; // Derivative: Considers the derivative of the change in error and impacts the output

        public static final double drivekP = 0.01; // Proportional: If there is error, move the motor proportional to the error
        public static final double drivekI = 0.00; // Integral: Creates a sum of the error over time. Increases until at set position
        public static final double drivekD = 0.00; // Derivative: Considers the derivative of the change in error and impacts the output

        /* Module Motor Characterization */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving. A small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.945);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.610);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.839);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.856);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static class SwerveConstants {
        /* Motor Hardware being use to use the right code */
        public static enum MotorHardware { NEO }
        public static MotorHardware SwerveHardware = MotorHardware.NEO; // Change this to the motor hardware you are using
        public static SwerveModuleGearing ModuleGearing = SwerveModuleGearing.MK4N_L1; // Change this to the module gearing you are using
        public static ModuleConfiguration SwerveModule = ModuleConfiguration.SwerveDriveSpecialities.MK4N.NEO(ModuleGearing.getDriveReduction()); // Change this to the motor configuration you are using

        /* Information of Drivetrain to perform kinematics */
        public static final double TrackWidth = Units.inchesToMeters(28); // Distance from center of the right wheels to center on the left wheels
        public static final double WheelBase = Units.inchesToMeters(28); // Distance from the center of the front wheels to center on the back wheels
        public static final double WheelDiameter = Units.inchesToMeters(4); // Diameter of the swerve pod wheel
        public static final double WheelCircumference = WheelDiameter * Math.PI;

        public static final double DriveGearing = (6.86 / 1.0);
        public static final double SteeringGearing = (18.75 / 1.0);

        /* Swerve Kinematics generated by defining the locations of each module from the center (origin) of the robot */
        public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, TrackWidth / 2),
                new Translation2d(WheelBase / 2, -TrackWidth / 2),
                new Translation2d(-WheelBase / 2, TrackWidth / 2),
                new Translation2d(-WheelBase / 2, -TrackWidth / 2)
        );

        /* Conversion Factor to convert the motor rotation to actual movement */
        public static final double DrivePositionConversionFactor = WheelCircumference / DriveGearing;
        public static final double DriveVelocityConversionFactor = DrivePositionConversionFactor / 60.0;
        public static final double SteeringPositionConversionFactor = 360 / SteeringGearing;

        /* Mechanical Constraints of Swerve Drive (for NEOs) */
        public static final double PhysicalMaxTranslationSpeed = 4.4; // Maximum speed in meters per second the gearing on the module allows you to go (different from free speed)
        public static final double PhysicalMaxTranslationAcceleration = 2; // Maxiumum speed in meters per second squared that the robot can realistically speed up to every second
        public static final double PhysicalMaxAngularVelocity = 3 * Math.PI; // Maximum speed in radians per second that the drivetrain will rotate.
        public static final double PhysicalMaxAngularAcceleration = Math.PI; // Maxiumum speed in radians per second squared that the robot can realistically speed up to every second

        /* Gyroscope Profiling */
        public static final double headingkP = 0.01; // Proportional: If there is error, move the motor proportional to the error
        public static final double headingkI = 0.00; // Integral: Creates a sum of the error over time. Increases until at set position
        public static final double headingkD = 0.00; // Derivative: Considers the derivative of the change in error and impacts the output

        /* Neutral Modes */
        public static final boolean activeNeutralMode = true; // What to do when neutral power is applied to the drivetrain while running
        public static final boolean disabledNeutralMode = false; // What to do when neutral power is applied to the drivetrain while disabled

        /* Motor and Encoder Inversions (should all be CCW+) */
        public static final boolean driveInverted = SwerveModule.driveMotorInverted;
        public static final boolean steeringInverted = SwerveModule.angleMotorInverted;
        public static final boolean swerveEncoderInverted = SwerveModule.cancoderInvert;
        public static final boolean gyroscopeInverted = false;
    }

    public static final class ControllerRawButtons {
        public static final class XboxController {
            public enum Axis {
                kLeftX(0),
                kLeftY(1),
                kLeftTrigger(2),
                kRightTrigger(3),
                kRightX(4),
                kRightY(5);

                public final int value;

                Axis(int value) {
                    this.value = value;
                }

                public String toString() {
                    var name = this.name().substring(1); // Remove leading `k`
                    if (name.endsWith("Trigger")) {
                        return name + "Axis";
                    }

                    return name;
                }
            }

            /* Button Constants for Xbox Controllers */
            public enum Button {
                kA(1),
                kB(2),
                kX(3),
                kY(4),
                kLeftBumper(5),
                kRightBumper(6),
                kBack(7),
                kStart(8),
                kLeftStick(9),
                kRightStick(10);

                public final int value;

                Button(int value) {
                    this.value = value;
                }

                public String toString() {
                    var name = this.name().substring(1); // Remove leading `k`
                    if (name.endsWith("Bumper")) {
                        return name;
                    }

                    return name + "Button";
                }
            }
        }

        /* DPAD Angular Constants for Xbox Controllers */
        public static final int DPAD_NORTH = 0;
        public static final int DPAD_NORTHEAST = 45;
        public static final int DPAD_EAST = 90;
        public static final int DPAD_SOUTHEAST = 135;
        public static final int DPAD_SOUTH = 180;
        public static final int DPAD_SOUTHWEST = 225;
        public static final int DPAD_WEST = 270;
        public static final int DPAD_NORTHWEST = 315;
        public static final int DPAD_NOT_PRESSED = -1;
    }

    public static class DriverConstants {
        public static final double kDeadband = 0.1;
    }
}
