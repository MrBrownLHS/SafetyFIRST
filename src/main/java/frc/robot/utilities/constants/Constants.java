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

    public static final class Lift {
        public static final int LIFT_MOTOR_ID = 13;
        public static final double LIFT_kP = 0.15;
        public static final double LIFT_kI = 0.0;
        public static final double LIFT_kD = 0.0;
        public static final double LIFT_kIZone = 5.0;
        public static final double LIFT_kG = 0.0;

        public static final double LIFT_MAX_VELOCITY = 20.0; 
        public static final double LIFT_MAX_ACCELERATION = 20.0; 
        public static final double LIFT_POSITION_TOLERANCE = 1.0; 

        public static final double LIFT_COLLECT_POS = 0.0;
        public static final double LIFT_L1_POS = -20.0;
        public static final double LIFT_L2_POS = -40.0;
        public static final double LIFT_L3_POS = -65.0;
        public static final double LIFT_CLIMB_POS = -75.0;
        public static final double LIFT_GEAR_RATIO = 1.0;
        public static final double LIFT_COG_DIAMETER_INCHES = 2.0;
        public static final double LIFT_ENCODER_TO_INCHES = (Math.PI * LIFT_COG_DIAMETER_INCHES) / 360.0 / LIFT_GEAR_RATIO;
    }

    public static final class Pivot {            
        public static final int PIVOT_MOTOR_ID = 15;
        public static final double PIVOT_kP = 0.2;
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.0;
        public static final double PIVOT_kIZone = 5.0;
        public static final double PIVOT_kG = 1.5;

        public static final double PIVOT_MAX_VELOCITY = 20.0; 
        public static final double PIVOT_MAX_ACCELERATION = 20.0;
        public static final double PIVOT_POSITION_TOLERANCE = 1.0; 

        public static final double PIVOT_COLLECT_POS = 0.0;
        public static final double PIVOT_L1_POS = -20.0;
        public static final double PIVOT_L2_POS = -30.0;
        public static final double PIVOT_L3_POS = -50.0;
        public static final double PIVOT_CLIMB_POS = -100.0;
        public static final double ENCODER_TO_DEGREES = 360.0 / 4096.0; 
    }

    public static final class Rotate {
        public static final int ROTATE_MOTOR_ID = 18;
        public static final double ROTATE_kP = 0.15;
        public static final double ROTATE_kI = 0.0;
        public static final double ROTATE_kD = 0.0;
        public static final double ROTATE_kIZone = 5.0;
        public static final double ROTATE_kG = 0.5;

        public static final double ROTATE_MAX_VELOCITY = 20.0;
        public static final double ROTATE_MAX_ACCELERATION = 20.0;

        public static final double ROTATE_POSITION_TOLERANCE = 0.5; 
        public static final double ROTATE_SPEED = 0.25;
        public static final double ROTATE_COLLECT_POS = 0.0;
        public static final double ROTATE_L1_POS = -5.0;
        public static final double ROTATE_L2_POS = -8.0;
        public static final double ROTATE_L3_POS = -10.0;
        public static final double ROTATE_CLIMB_POS = -20.0;
        public static final double ROTATE_DEADBAND = 0.1; 
        public static final double ROTATE_RATE_LIMIT = 2.9; 
    }

    public static final class Intake {
        public static final int CORAL_INTAKE_MOTOR_ID = 17;
        public static final int CORAL_LIMIT_SWITCH_PORT = 1;
        public static final double CORAL_INTAKE_SPEED = 0.25; 
        public static final double CORAL_OUTTAKE_SPEED = -0.25; 
        public static final double CORAL_AUTO_RELEASE_SPEED = -0.25;
        public static final double CORAL_INTAKE_RATE_LIMIT = 2.9; 
    }
             
    public static final class MotorConstants {
        public static final double VOLTAGE_COMPENSATION = 12.0;
        public static final int CURRENT_LIMIT_NEO = 60;
        public static final int CURRENT_THRESHOLD_NEO = 60;
        public static final double CURRENT_THRESHOLD_TIME_NEO = 0.1;
        public static final int MAX_CURRENT_LIMIT_NEO = 60;
        public static final boolean ENABLE_CURRENT_LIMIT_NEO = true;

        public static final int CURRENT_LIMIT_550 = 20;
        public static final int CURRENT_THRESHOLD_550 = 40;
        public static final double CURRENT_THRESHOLD_TIME_550 = 10;
        public static final int MAX_CURRENT_LIMIT_550 = 60;
        public static final boolean ENABLE_CURRENT_LIMIT_550 = true;
        public static final boolean ACTIVE_NEUTRAL_MODE = true; 
        public static final boolean DISABLE_NEUTRAL_MODE = false;
    }

    public static final class CageClimberConstants {
        public static final int WINCH_MOTOR_ID = 19;
        public static final double WINCH_MOTOR_SPEED = 0.5;
    }

    public static final class AlgaeCollectorConstants {
        public static final int ALGAE_ARTICULATE_MOTOR_ID = 20;
        public static final int ALGAE_CLAW_MOTOR_ID= 21;
        public static final double ALGAE_ARTICULATE_DEADBAND = 0.1;
        public static final double ALGAE_ARTICULATE_RATE_LIMIT = 2.9; 
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
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.4436);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.838379);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.6059);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.357178);
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
        public static final boolean disabledNeutralMode = true; // What to do when neutral power is applied to the drivetrain while disabled

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

