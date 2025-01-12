package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Robot;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.SwerveModuleConstants;

public class NEOSwerveModule {
    public int moduleNumber;
    public Rotation2d lastAngle;
    public Rotation2d angleOffset;

    private SwerveModuleState expectedState = new SwerveModuleState();

    private SparkMax driveMotor;
    private SparkMax steeringMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder steeringEncoder;
    private CANcoder swerveEncoder;
    private CANcoderConfigurator swerveEncoderConfigurator;

    private final SparkClosedLoopController drivePIDController;
    private final SparkClosedLoopController steeringPIDController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ModuleConstants.driveKS, Constants.ModuleConstants.driveKV, Constants.ModuleConstants.driveKA);

    public NEOSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        swerveEncoder = new CANcoder(moduleConstants.swerveEnocderID);
        configureSwerveEnocder();

        steeringMotor = new SparkMax(moduleConstants.steeringMotorID, MotorType.kBrushless);
        steeringEncoder = steeringMotor.getEncoder();
        steeringPIDController = steeringMotor.getClosedLoopController();
        configureSteeringMotor();

        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        //driveMotor.setInverted(Constants);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getClosedLoopController();
        configureDriveMotor();

        //lastAngle =
    }

    private void configureSwerveEnocder() {
        swerveEncoderConfigurator = swerveEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();

        magnetSensorConfiguration.AbsoluteSensorDiscontinuityPoint = 1;
        magnetSensorConfiguration.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        magnetSensorConfiguration.MagnetOffset = angleOffset.getRotations();
        swerveEncoderConfigurator.apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfiguration));
    }

    private void configureSteeringMotor() {
        var config = new SparkMaxConfig();
        config.smartCurrentLimit(Constants.ModuleConstants.steeringCurrentLimit);
        config.secondaryCurrentLimit(Constants.ModuleConstants.maximumCurrentLimit);
        config.inverted(Constants.SwerveConstants.steeringInverted);
        config.idleMode(Constants.SwerveConstants.disabledNeutralMode ? IdleMode.kCoast : IdleMode.kBrake);
        config.encoder.positionConversionFactor(Constants.SwerveConstants.SteeringPositionConversionFactor);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(Constants.ModuleConstants.steeringkP, Constants.ModuleConstants.steeringkI, Constants.ModuleConstants.steeringkD);
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(0, 360);
        config.voltageCompensation(Constants.ModuleConstants.voltageCompensation);
        steeringMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetToAbsolute();
    }

    private void configureDriveMotor() {
        var config = new SparkMaxConfig();
        config.smartCurrentLimit(Constants.ModuleConstants.driveCurrentLimitNEO);
        config.secondaryCurrentLimit(Constants.ModuleConstants.maximumCurrentLimit);
        config.idleMode(Constants.SwerveConstants.disabledNeutralMode ? IdleMode.kCoast : IdleMode.kBrake);
        config.encoder.positionConversionFactor(Constants.SwerveConstants.DrivePositionConversionFactor);
        config.encoder.velocityConversionFactor(Constants.SwerveConstants.DriveVelocityConversionFactor);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(Constants.ModuleConstants.drivekP, Constants.ModuleConstants.drivekI, Constants.ModuleConstants.drivekD);
        config.voltageCompensation(Constants.ModuleConstants.voltageCompensation);
        driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveEncoder.setPosition(0.0);
    }

    public Rotation2d getSwerveEncoder() {
        return Rotation2d.fromRotations(swerveEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getSteeringAngle() {
        return Rotation2d.fromDegrees(steeringEncoder.getPosition());
    }

    public double getMotorVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public SwerveModuleState getDesiredState() {
        return expectedState;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getMotorVelocity(), getSteeringAngle());
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getSteeringAngle());
    }

    public void resetToAbsolute() {
        double absolutePosition = getSwerveEncoder().getDegrees();
        steeringEncoder.setPosition(absolutePosition);
    }

    public void onDisabled() {
        var config = new SparkMaxConfig();
        config.idleMode(Constants.SwerveConstants.disabledNeutralMode ? IdleMode.kCoast : IdleMode.kBrake);
        driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        steeringMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void onEnabled() {
        var config = new SparkMaxConfig();
        config.idleMode(Constants.SwerveConstants.activeNeutralMode ? IdleMode.kCoast : IdleMode.kBrake);
        driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        steeringMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.PhysicalMaxTranslationSpeed;
            driveMotor.set(percentOutput);
        } else {
            drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        steeringPIDController.setReference(MathUtil.inputModulus(desiredState.angle.getDegrees(), 0, 360), ControlType.kPosition);

        if(Robot.isSimulation()) {
            steeringEncoder.setPosition(desiredState.angle.getDegrees());
        }

        // lastAngle = angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(Math.abs(desiredState.speedMetersPerSecond) < 0.006) {
            driveMotor.set(0);
            steeringMotor.set(0);

            if(desiredState.angle == lastAngle) {
                resetToAbsolute();
            }

            return;
        }

        desiredState = OnboardModuleState.optimize(desiredState, getSwerveModuleState().angle);
        expectedState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void voltageDrive(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    public void stopDriveMotor() {
        driveMotor.set(0.0);
    }

    public void stopSteeringMotor() {
        steeringMotor.set(0.0);
    }

    public void stop() {
        stopDriveMotor();
        stopSteeringMotor();
    }
}