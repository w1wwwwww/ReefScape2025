package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class SwerveModule {

    private final SparkMax DriveMotor;
    private final SparkMax AngleMotor;
    private final RelativeEncoder DriveEncoder;
    private final CANcoder AngleEncoder;
    private final CANcoderConfiguration AngleEncoderConfiguration;

    public SwerveModule(int driveMotorID, int angleMotorID, int CANCoderID){

        DriveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        DriveEncoder = DriveMotor.getEncoder();

        AngleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        AngleEncoder = new CANcoder(CANCoderID);
        AngleEncoderConfiguration = CANcoderConfiguration();

    }
    
}
