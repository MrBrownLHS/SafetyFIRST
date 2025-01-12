package frc.robot.utilities;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

public class ModuleConfiguration {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final boolean driveMotorInverted;
    public final boolean angleMotorInverted;
    public final boolean cancoderInvert;

    public ModuleConfiguration(double wheelDiameter, double angleGearRatio, double driveGearRatio, boolean driveMotorInverted, boolean angleMotorInverted, boolean cancoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;
        this.cancoderInvert = cancoderInvert;
    }

    public static final class SwerveDriveSpecialities {
        public static final class MK4N {
            
            public static final ModuleConfiguration NEO(double driveGearRatio) {
                double wheelDiameter = Units.inchesToMeters(4.0);
                double angleGearRatio = SwerveModuleGearing.MK4N_L1.getSteerReduction();

                boolean driveMotorInvert = SwerveModuleGearing.MK4N_L1.isDriveInverted();
                boolean angleMotorInvert = SwerveModuleGearing.MK4N_L1.isSteerInverted();
                boolean cancoderInvert = false;

                return new ModuleConfiguration(wheelDiameter, angleGearRatio, driveGearRatio, driveMotorInvert, angleMotorInvert, cancoderInvert);
            }

            public static final class RatioOptions {
                public static final double L1 = SwerveModuleGearing.MK4N_L1.getDriveReduction();
                public static final double L2 = SwerveModuleGearing.MK4N_L2.getDriveReduction();
                public static final double L3 = SwerveModuleGearing.MK4N_L3.getDriveReduction();
            }
        }
    }
}