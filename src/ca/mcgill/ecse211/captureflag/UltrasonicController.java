package ca.mcgill.ecse211.captureflag;

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
