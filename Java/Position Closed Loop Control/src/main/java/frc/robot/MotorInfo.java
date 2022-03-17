package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTableEntry;

public class MotorInfo {

    public CANSparkMax motor;
    public int id;
    public RelativeEncoder encoder;
    public SparkMaxPIDController pidController;
    public String name;
    public NetworkTableEntry ntRotTarget;
    public NetworkTableEntry ntRotCurrent;
    public NetworkTableEntry ntEnable;

}
