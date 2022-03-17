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

  // Initial PID coefficients
  private double kP = 0.1;
  private double kI = 1e-4;
  private double kD = 1;
  private double kIz = 0;
  private double kFF = 0;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;

  private ArrayList<MotorInfo> motors;
  private ShuffleboardTab shuffTab = Shuffleboard.getTab("SparkMax");
  private NetworkTableEntry ntEntryP, ntEntryI, ntEntryD, ntEntryIz,
          ntEntryFF, ntEntryMaxOut, ntEntryMinOut,
          driveGo, driveRotations;
  private boolean lastGo = false;

  private MotorInfo motorInit(String name, int canID) {
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

    MotorInfo motorInfo = new MotorInfo();
    motorInfo.motor = m;
    motorInfo.encoder = encoder;
    motorInfo.pidController = pidc;
    motorInfo.id = canID;
    motorInfo.name = name;

    ShuffleboardLayout motorGroup = shuffTab.getLayout(name + "(" + canID + ")", BuiltInLayouts.kList).withSize(1, 2);
    motorInfo.ntRotCurrent = motorGroup.add("Rotations", encoder.getPosition()).getEntry();
    motorInfo.ntRotTarget = motorGroup.add("Target", encoder.getPosition()).getEntry();

    return motorInfo;
  }


  @Override
  public void robotInit() {
    motors = new ArrayList<MotorInfo>();

    ShuffleboardLayout pidGroup = shuffTab.getLayout("PID", BuiltInLayouts.kList).withSize(2, 4);
    ntEntryP = pidGroup.add("P Gain", kP).getEntry();
    ntEntryI = pidGroup.add("I Gain", kI).getEntry();
    ntEntryD = pidGroup.add("D Gain", kD).getEntry();
    ntEntryIz = pidGroup.add("I Zone", kIz).getEntry();
    ntEntryFF = pidGroup.add("FeedForward", kFF).getEntry();
    ntEntryMaxOut = pidGroup.add("MaxOut", kMaxOutput).
            withWidget(BuiltInWidgets.kNumberSlider).
            withProperties(Map.of("min", -1, "max", 0)).getEntry();
    ntEntryMinOut = pidGroup.add("MinOut", kMinOutput).
            withWidget(BuiltInWidgets.kNumberSlider).
            withProperties(Map.of("min", 0, "max", 1)).getEntry();


    ShuffleboardLayout driveGroup = shuffTab.getLayout("Drive", BuiltInLayouts.kList).withSize(1, 2);
    driveGo = driveGroup.add("Go", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    driveRotations = driveGroup.add("Rotations", 0).getEntry();

    motors.add(motorInit("LFront", lFrontID));
    motors.add(motorInit("LRear", lRearID));
    motors.add(motorInit("RFront", rFrontID));
    motors.add(motorInit("RRear", rRearID));
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
        for (int i=0; i<motors.size(); i++) {
            motors.get(i).pidController.setP(xP);
        }
    }

    if (xI != kI) {
        kI = xI;
        for (int i=0; i<motors.size(); i++) {
            motors.get(i).pidController.setI(xI);
        }
    }

    if (xD != kD) {
        kD = xD;
        for (int i=0; i<motors.size(); i++) {
            motors.get(i).pidController.setD(xD);
        }
    }

    if (xIz != kIz) {
        kIz = xIz;
        for (int i=0; i<motors.size(); i++) {
            motors.get(i).pidController.setIZone(xIz);
        }
    }

    if (xFF != kFF) {
        kFF = xFF;
        for (int i=0; i<motors.size(); i++) {
            motors.get(i).pidController.setFF(xFF);
        }
    }

    if ((xMax != kMaxOutput) || (xMin != kMaxOutput)) {
        kMinOutput = xMax;
        kMaxOutput = xMax;
        for (int i=0; i<motors.size(); i++) {
            motors.get(i).pidController.setOutputRange(xMin, xMax);
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
    MotorInfo m;
    if (curGo == lastGo) {
        // no change. We start or stop on Go toggle;
        for (int i=0; i<motors.size(); i++) {
            m = motors.get(i);
            m.ntRotCurrent.setDouble(m.encoder.getPosition());
        }
    } else {
        double rotations = driveRotations.getDouble(0.0f);
        double targetRot, currentRot;
        if (curGo) {
            // go
            updatePidControllers();
            for(int i=0; i<motors.size(); i++) {
                m = motors.get(i);
                currentRot = m.encoder.getPosition();
                targetRot = currentRot + rotations;
                m.pidController.setReference(targetRot, CANSparkMax.ControlType.kPosition);
                m.ntRotCurrent.setDouble(currentRot);
                m.ntRotTarget.setDouble(targetRot);
            }
        } else {
            // stop;
            for(int i=0; i<motors.size(); i++) {
                m = motors.get(i);
                currentRot = m.encoder.getPosition() + rotations;
                targetRot = currentRot;
                m.pidController.setReference(currentRot, CANSparkMax.ControlType.kPosition);
                m.ntRotCurrent.setDouble(currentRot);
                m.ntRotTarget.setDouble(targetRot);
            }
        }
        lastGo = curGo;
    }

  }

  @Override
  public void teleopPeriodic() {
  }
}
