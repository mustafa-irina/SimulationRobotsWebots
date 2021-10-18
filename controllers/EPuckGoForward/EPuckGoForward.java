// File:          EPuckGoForward.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class EPuckGoForward {
  public static void main(String[] args) {
  
     int TIME_STEP = 64;
  
     double MAX_SPEED = 6.28;
  
     Robot robot = new Robot();
  
     // get a handler to the motors and set target position to infinity (speed control)
     Motor leftMotor = robot.getMotor("left wheel motor");
     Motor rightMotor = robot.getMotor("right wheel motor");
     leftMotor.setPosition(Double.POSITIVE_INFINITY);
     rightMotor.setPosition(Double.POSITIVE_INFINITY);
  
     // set up the motor speeds at 10% of the MAX_SPEED.
     leftMotor.setVelocity(0.1 * MAX_SPEED);
     rightMotor.setVelocity(0.1 * MAX_SPEED);
  
     while (robot.step(TIME_STEP) != -1);
   }
}
