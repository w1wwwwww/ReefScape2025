package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

public class SwerveModule {

    private SparkMax DriveMotor;
    private SparkMax AngleMotor;
    private RelativeEncoder DriveEncoder;
    private CANcoder AngleEncoder;
    private CANcoderConfiguration AngleEncoderConfiguration;
    private double AngleEncoderOffset;
    private SparkMaxConfig DriveMotorConfig;

    public SwerveModule(int driveMotorID, int angleMotorID, int CANCoderID, double AngleEncoderOffset){

        DriveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);

        DriveEncoder = DriveMotor.getEncoder();

        AngleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        AngleEncoder = new CANcoder(CANCoderID);

        AngleEncoderConfiguration = new CANcoderConfiguration();

        AngleEncoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Constants.ABSOLUTE_ENCODER_DISCONTINUITY_POINT);
        AngleEncoderConfiguration.MagnetSensor.MagnetOffset = AngleEncoderOffset;
        AngleEncoderConfiguration.MagnetSensor.SensorDirection = Constants.ABSOLUTE_ENCODER_SENSOR_DIRECTION;

        AngleEncoder.getConfigurator().apply(AngleEncoderConfiguration);

    }
    
}
