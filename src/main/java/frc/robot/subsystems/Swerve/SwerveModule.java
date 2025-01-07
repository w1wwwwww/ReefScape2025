package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
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

    //Constructor that allows for all of the modules to be created in the subsytem by feeding in the ids and offsets
    public SwerveModule(int driveMotorID, int angleMotorID, int CANCoderID, double AngleEncoderOffset){

        //Defines the motor in the constructor to tell the rest of the clas it exists
        DriveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);

        //Defines the drive encoder of off the drive motor to tell us how many times the robot has spun
        DriveEncoder = DriveMotor.getEncoder();

        //Defines the motor and the CANCoder
        AngleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        AngleEncoder = new CANcoder(CANCoderID);

        //Makes a configurator object for the CANCoder allowing us to change specific parts of it
        AngleEncoderConfiguration = new CANcoderConfiguration();

        //Changes the magnet sensor settings to work better with our robot
        AngleEncoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Constants.ABSOLUTE_ENCODER_DISCONTINUITY_POINT);
        AngleEncoderConfiguration.MagnetSensor.MagnetOffset = AngleEncoderOffset;
        AngleEncoderConfiguration.MagnetSensor.SensorDirection = Constants.ABSOLUTE_ENCODER_SENSOR_DIRECTION;

        //applies the changes that were made to the CANCoder
        AngleEncoder.getConfigurator().apply(AngleEncoderConfiguration);

    }

    //Gets the state of this module by giving you the wheel veloicty and the value of the wheel angle
    public SwerveModuleState getState(){
        return new SwerveModuleState(DriveEncoder.getVelocity(), new Rotation2d(AngleEncoder.getAbsolutePosition().getValue()));
    }

    //Gets the current position of the robot or where it is
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), new Rotation2d(AngleEncoder.getAbsolutePosition().getValue()));
    }

    //Sets the desired wheel state of this module for the robot
    public void setDesiredStates(SwerveModuleState state){
        //used to prevent the robot wheels from spinning further thatn 90 degrees
        state.optimize(null); //needs a get angle function
    }
    
}
