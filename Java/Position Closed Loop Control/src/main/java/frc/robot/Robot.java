/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private static final int lFrontID = 1;
  private static final int lRearID = 2;
  private static final int rFrontID = 4;
  private static final int rRearID = 3;
  private CANSparkMax lFrontMotor, lRearMotor, rFrontMotor, rRearMotor;
  private ShuffleboardTab shuffTab = Shuffleboard.getTab("SparkMax");
  private NetworkTableEntry ntEntryP, ntEntryI, ntEntryD, ntEntryIz,
          ntEntryFF, ntEntryMaxOut, ntEntryMinOut,
          driveGo, driveRotations;
  private boolean lastGo = false;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private ArrayList<CANSparkMax> motors = new ArrayList<CANSparkMax>();
  private ArrayList<SparkMaxPIDController> pidControllers = new ArrayList<SparkMaxPIDController>();
  private ArrayList<String> names = new ArrayList<String>();

  private CANSparkMax motorInit(String name, int canID) {
    // initialize motor
    CANSparkMax m = new CANSparkMax(canID, MotorType.kBrushless);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    SparkMaxPIDController pidc = m.getPIDController();

    // Encoder object created to display position values
    RelativeEncoder encoder = m.getEncoder();

    // set PID coefficients
    pidc.setP(kP);
    pidc.setI(kI);
    pidc.setD(kD);
    pidc.setIZone(kIz);
    pidc.setFF(kFF);
    pidc.setOutputRange(kMinOutput, kMaxOutput);

    // SmartDashboard.putNumber("Set Rotations", 0);
    ShuffleboardLayout motorGroup = shuffTab.getLayout(name, BuiltInLayouts.kList);
    motorGroup.add("Rotations", encoder.getPosition());
    motorGroup.add("Target", encoder.getPosition());

    return m;
  }


  @Override
  public void robotInit() {
    // PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    ShuffleboardLayout pidGroup = shuffTab.getLayout("PID", BuiltInLayouts.kList);
    ntEntryP = pidGroup.add("P Gain", kP).getEntry();
    ntEntryI = pidGroup.add("I Gain", kI).getEntry();
    ntEntryD = pidGroup.add("D Gain", kD).getEntry();
    ntEntryIz = pidGroup.add("I Zone", kIz).getEntry();
    ntEntryFF = pidGroup.add("FeedForward", kFF).getEntry();
    ntEntryMaxOut = pidGroup.add("MaxOut", kMaxOutput).
            withWidget(BuiltInWidgets.kNumberSlider).
            withProperties(Map.of("min", 0, "max", 1)).getEntry();
    ntEntryMinOut = pidGroup.add("MinOut", kMinOutput).
            withWidget(BuiltInWidgets.kNumberSlider).
            withProperties(Map.of("min", 0, "max", 1)).getEntry();


    ShuffleboardLayout driveGroup = shuffTab.getLayout("Drive", BuiltInLayouts.kList);
    driveGo = driveGroup.add("Go", false).getEntry();
    driveRotations = driveGroup.add("Rotations", 0).getEntry();

    lFrontMotor = motorInit("LFront", lFrontID);
    lRearMotor = motorInit("LRear", lRearID);
    rFrontMotor = motorInit("RFront", rFrontID);
    rRearMotor = motorInit("RRear", rRearID);

    motors.add(lFrontMotor); names.add("LFront");
    motors.add(lRearMotor);  names.add("LRear");
    motors.add(rFrontMotor); names.add("RFront");
    motors.add(rRearMotor);  names.add("RRear");

    for (int i=0; i<motors.size(); i++) {
      pidControllers.add(motors.get(i).getPIDController());
    }
  }

  private void updatePidControllers() {
    // read PID coefficients from SmartDashboard
    double xP = ntEntryP.getDouble(0.0);
    double xI = ntEntryI.getDouble(0.0);
    double xD = ntEntryD.getDouble(0.0);
    double xIz = ntEntryIz.getDouble(0.0);
    double xFF = ntEntryFF.getDouble(0.0);
    double xMax = ntEntryMaxOut.getDouble(0.0);
    double xMin = ntEntryMinOut.getDouble(0.0);
    // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    if (xP != kP) {
        kP = xP;
        for (int i=0; i<pidControllers.size(); i++) {
            pidControllers.get(i).setP(xP);
        }
    }

    if (xI != kI) {
        kI = xI;
        for (int i=0; i<pidControllers.size(); i++) {
            pidControllers.get(i).setI(xI);
        }
    }

    if (xD != kD) {
        kD = xD;
        for (int i=0; i<pidControllers.size(); i++) {
            pidControllers.get(i).setD(xD);
        }
    }

    if (xIz != kIz) {
        kIz = xIz;
        for (int i=0; i<pidControllers.size(); i++) {
            pidControllers.get(i).setIZone(xIz);
        }
    }

    if (xFF != kFF) {
        kFF = xFF;
        for (int i=0; i<pidControllers.size(); i++) {
            pidControllers.get(i).setFF(xFF);
        }
    }

    if ((xMax != kMaxOutput) || (xMin != kMaxOutput)) {
        kMinOutput = xMax;
        kMaxOutput = xMax;
        for (int i=0; i<pidControllers.size(); i++) {
            pidControllers.get(i).setOutputRange(xMin, xMax);
        }
    }
  }

  @Override
  public void autonomousPeriodic() {
    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     *
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     *
     * The second parameter is the control type can be set to one of four
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    boolean curGo = driveGo.getBoolean(false);
    if (curGo == lastGo) {
        // no change. We start or stop on Go toggle;
        RelativeEncoder enc;
        for (int i=0; i<motors.size(); i++) {
            enc = motors.get(i).getEncoder();
	    // TODO: update the motors current shuffleboard value.
        }
    } else {
        double rotations = driveRotations.getDouble(0.0f);
        updatePidControllers();
        RelativeEncoder enc;
        if (curGo) {
            for(int i=0; i<pidControllers.size(); i++) {
                enc = motors.get(i).getEncoder();
                double target = enc.getPosition() + rotations;
                pidControllers.get(i).setReference(target, CANSparkMax.ControlType.kPosition);
                // TODO: update the motor's target shuffleboard Value.
            }
        } else {
            for(int i=0; i<pidControllers.size(); i++) {
                enc = motors.get(i).getEncoder();
                pidControllers.get(i).setReference(enc.getPosition(), CANSparkMax.ControlType.kPosition);
                // TODO: update the motor's target shuffleboard Value.
            }
        }
        lastGo = curGo;
        // m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        // SmartDashboard.putNumber("SetPoint", rotations);
        // SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
    }

  }

  @Override
  public void teleopPeriodic() {
  }
}
