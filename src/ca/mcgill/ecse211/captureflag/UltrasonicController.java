package ca.mcgill.ecse211.captureflag;

/**
 * This class acts as an interface that defines method signatures for 
 * processing ultrasonic data and reading ultrasonic distance
 */
public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
