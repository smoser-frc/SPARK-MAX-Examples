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
  private double kMaxOutput = 0.5;
  private double kMinOutput = -0.5;

  private ArrayList<MotorInfo> motors;
  private ShuffleboardTab shuffTab = Shuffleboard.getTab("SparkMax");
  private NetworkTableEntry ntEntryP, ntEntryI, ntEntryD, ntEntryIz,
          ntEntryFF, ntEntryMaxOut, ntEntryMinOut,
          driveGo, driveRotations;
  private boolean lastGo = false;

  private MotorInfo motorInit(String name, int canID, boolean invert) {
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

    ShuffleboardLayout motorGroup = shuffTab.
        getLayout(name + "(" + canID + ")", BuiltInLayouts.kGrid).withSize(1, 3);
    motorInfo.ntEnable = motorGroup.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).
        withPosition(0, 0).getEntry();
    motorInfo.ntInvert = motorGroup.add("Invert", invert).withWidget(BuiltInWidgets.kToggleSwitch).
        withPosition(0, 1).getEntry();
    motorInfo.ntRotCurrent = motorGroup.add("Rotations", encoder.getPosition()).
        withPosition(0, 2).getEntry();
    motorInfo.ntRotTarget = motorGroup.add("Target", encoder.getPosition()).
        withPosition(0, 3).getEntry();

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
    ntEntryMinOut = pidGroup.add("MinOut", kMinOutput).
            withWidget(BuiltInWidgets.kNumberSlider).
            withProperties(Map.of("min", -1, "max", 0)).getEntry();
    ntEntryMaxOut = pidGroup.add("MaxOut", kMaxOutput).
            withWidget(BuiltInWidgets.kNumberSlider).
            withProperties(Map.of("min", 0, "max", 1)).getEntry();


    ShuffleboardLayout driveGroup = shuffTab.getLayout("Drive", BuiltInLayouts.kList).withSize(1, 2);
    driveGo = driveGroup.add("Go", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    driveRotations = driveGroup.add("Rotations", 0).getEntry();

    motors.add(motorInit("LFront", lFrontID, false));
    motors.add(motorInit("LRear", lRearID, false));
    motors.add(motorInit("RFront", rFrontID, false));
    motors.add(motorInit("RRear", rRearID, false));
  }

  // this is called once per 'Go'
  private void updatePidControllers() {
    // read PID coefficients from SmartDashboard
    double xP = ntEntryP.getDouble(0.0);
    double xI = ntEntryI.getDouble(0.0);
    double xD = ntEntryD.getDouble(0.0);
    double xIz = ntEntryIz.getDouble(0.0);
    double xFF = ntEntryFF.getDouble(0.0);
    double xMax = ntEntryMaxOut.getDouble(0.0);
    double xMin = ntEntryMinOut.getDouble(0.0);

    boolean changeP = (xP == kP);
    boolean changeI = (xI == kI);
    boolean changeD = (xD == kD);
    boolean changeIz = (xIz == kIz);
    boolean changeFF = (xFF == kFF);
    boolean changeMax = (xMax == kMaxOutput);
    boolean changeMin = (xMin == kMinOutput);

    MotorInfo m;
    SparkMaxPIDController p;

    for (int i=0; i<motors.size(); i++) {
        m = motors.get(i);
        p = m.pidController;
        // zero out the encoder counter
        m.encoder.setPosition(0);

        if (!m.ntEnable.getBoolean(true)) {
            continue;
        }
        m.motor.setInverted(m.ntInvert.getBoolean(false));

        if (changeP) {
            kP = xP;
            p.setP(xP);
        }
        if (changeI) {
            kI = xI;
            p.setI(xI);
        }
        if (changeD) {
            kD = xD;
            p.setD(xD);
        }
        if (changeIz) {
            kIz = xIz;
            p.setIZone(xIz);
        }
        if (changeFF) {
            kFF = xFF;
            p.setFF(xFF);
        }
        if (changeMax || changeMin) {
            kMinOutput = xMax;
            kMaxOutput = xMax;
            p.setOutputRange(xMin, xMax);
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
                if (!m.ntEnable.getBoolean(true)) {
                    continue;
                }
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
                currentRot = m.encoder.getPosition();
                // cancel the operation.  Another option is: m.motor.set(0);
                m.pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
                m.ntRotCurrent.setDouble(currentRot);
                if (!m.ntEnable.getBoolean(true)) {
                    continue;
                }
                targetRot = currentRot;
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
