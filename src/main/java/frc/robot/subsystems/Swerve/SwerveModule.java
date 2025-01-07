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

    private SparkMax driveMotor;
    private SparkMax angleMotor;
    private RelativeEncoder driveEncoder;
    private CANcoder angleEncoder;
    private CANcoderConfiguration angleEncoderConfiguration;
    private double angleEncoderOffset;
    private SparkMaxConfig driveMotorConfig;
    private double angleAsDouble;
    private double moduleAngle;

    //Constructor that allows for all of the modules to be created in the subsytem by feeding in the ids and offsets
    public SwerveModule(int driveMotorID, int angleMotorID, int CANCoderID, double AngleEncoderOffset){

        //Defines the motor in the constructor to tell the rest of the clas it exists
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);

        //Defines the drive encoder of off the drive motor to tell us how many times the robot has spun
        driveEncoder = driveMotor.getEncoder();

        //Defines the motor and the CANCoder
        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = new CANcoder(CANCoderID);

        //Makes a configurator object for the CANCoder allowing us to change specific parts of it
        angleEncoderConfiguration = new CANcoderConfiguration();

        //Changes the magnet sensor settings to work better with our robot
        angleEncoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Constants.ABSOLUTE_ENCODER_DISCONTINUITY_POINT);
        angleEncoderConfiguration.MagnetSensor.MagnetOffset = AngleEncoderOffset;
        angleEncoderConfiguration.MagnetSensor.SensorDirection = Constants.ABSOLUTE_ENCODER_SENSOR_DIRECTION;

        //applies the changes that were made to the CANCoder
        angleEncoder.getConfigurator().apply(angleEncoderConfiguration);
        
    }

    //Returns the module angle in degrees
    public Rotation2d getAngle(){
        //Gets the CTRE value from -0.5 to 0.5
        double angleAsDouble = angleEncoder.getAbsolutePosition().getValueAsDouble();
        //Multiplies the value of -0.5 to 0.5 giving us the value as an angle
        double moduleAngle = 360 * angleAsDouble;
        return new Rotation2d(moduleAngle);
    }

    //Gets the state of this module by giving you the wheel veloicty and the value of the wheel angle
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    //Gets the current position of the robot or where it is
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    //Sets the desired wheel state of this module for the robot
    public void setDesiredStates(SwerveModuleState state){
        //used to prevent the robot wheels from spinning further that 90 degrees
        state.optimize(getAngle()); 
    }
    
}
