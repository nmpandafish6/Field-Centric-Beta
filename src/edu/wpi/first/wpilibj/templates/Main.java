/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Main extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    
    CANJaguar frontLeft;
    CANJaguar frontRight;
    CANJaguar backRight;
    CANJaguar backLeft;
    Joystick joy;
    Joystick joy2;
    Gyro gyro;
    double gyroAngle = 0, staticAngle = 0;
    
    public void robotInit() {
        try {
            frontLeft = new CANJaguar(3);
            frontRight = new CANJaguar(4);
            backRight = new CANJaguar(5);
            backLeft = new CANJaguar(2);
            joy = new Joystick(1);
            joy2 = new Joystick(2);
            gyro = new Gyro(2);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    }

    /**
     * This function is called periodically during operator control
     */
    double gyroConstant = -.5;
    public void teleopPeriodic() {
        
        if(joy.getRawButton(4)) {
            gyro.reset();
        }
        
        SmartDashboard.putNumber("Robot Angle", gyroAngle);
        gyroAngle = gyro.getAngle();
        double gyroAngleRads = gyroAngle * Math.PI / 180 * gyroConstant;
        double desiredAngle = (MathUtils.atan2(-joy.getY(), joy.getX()) + 3*Math.PI/2) % (2*Math.PI);
        double relativeAngle = (-(gyroAngleRads) + (desiredAngle) + (Math.PI/2)) % (2*Math.PI);
        double forward = Math.sin(relativeAngle);
        double strafe = Math.cos(relativeAngle);
        SmartDashboard.putNumber("Desired Angle", desiredAngle );
        SmartDashboard.putNumber("Relative Angle", relativeAngle);
        double rotate = joy2.getX();
        double unscaledJoy[] = {Math.sin(desiredAngle), Math.cos(desiredAngle)};
        double maxJoy[] = normalize(unscaledJoy, true);
        double scalar = threshhold((sqr(joyY) + sqr(joyX)) / (sqr(maxJoy[0]) + sqr(maxJoy[1])));
        SmartDashboard.putNumber("Scalar", scalar);
        double kP = 0.00277778;
        boolean update = false;
        
        double ftLeft = (forward + strafe)*scalar + rotate;
        double ftRight = (-forward + strafe)*scalar + rotate;
        double bkLeft = (forward - strafe)*scalar + rotate;
        double bkRight = (-forward - strafe)*scalar + rotate;
        
        SmartDashboard.putNumber("Strafe",strafe);
        SmartDashboard.putNumber("Forward",forward);
        
        double unnormalizedValues[] = {ftLeft, ftRight, bkLeft, bkRight};
        double output[] = normalize(unnormalizedValues, false);
        
        ftLeft = output[0];
        ftRight = output[1];
        bkLeft = output[2];
        bkRight = output[3];
        
        SmartDashboard.putNumber("Front Left" , ftLeft);
        SmartDashboard.putNumber("Front Right" , ftRight);
        SmartDashboard.putNumber("Back Left" , bkLeft);
        SmartDashboard.putNumber("Back Right" , bkRight);
        
        try{
            frontLeft.set(ftLeft);
            frontRight.set(ftRight);
            backLeft.set(bkLeft);
            backRight.set(bkRight);
        }catch(Exception e){
            System.out.println("Hey Listen");
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    public double sqr(double value){
        return value*value;
    }
    
    public static double threshhold(double value){
        if(value > 0){
            return Math.min(value, 1);
        }else{
            return Math.max(value, -1);
        }
    }
    
    public double[] normalize(double[] values, boolean scaleUp){
        double[] normalizedValues = new double[values.length];
        double max = Math.max(Math.abs(values[0]), Math.abs(values[1]));
        for(int i = 2; i < values.length; i++){
            max = Math.max(Math.abs(values[i]), max);
        }
        if(max < 1 && scaleUp == false) {
            for(int i = 0; i < values.length; i++){
                normalizedValues[i] = values[i];
            }
        }   else    {
            for(int i = 0; i < values.length; i++){
                normalizedValues[i] = values[i] / max;
            }
        }
        
        return normalizedValues;
    }
    
}
