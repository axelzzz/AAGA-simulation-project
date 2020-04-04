package algorithms;

import java.util.ArrayList;

import characteristics.IFrontSensorResult;
import characteristics.IFrontSensorResult.Types;
import characteristics.Parameters.Direction;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

public class FifthElementSecondary extends Brain {

	//---PARAMETERS---//
	private static final double HEADINGPRECISION = 0.001;
	private static final double ANGLEPRECISION = 0.1;
	private static final int BOTRADAR1 = 1;
	private static final int BOTRADAR2 = 2;

	//---VARIABLES---//
	private int counter;
	private boolean avoidObstacle, startingMovesOver;
	private double endTaskDirection;
	private Direction avoidingDirection;
	private int whoAmI;
	private IFrontSensorResult.Types obstacle;

	public FifthElementSecondary() {
		super();
	}
	
	public void activate() {
		identifyBots();
		if (whoAmI == BOTRADAR1) {
			avoidingDirection = Parameters.Direction.RIGHT;
			endTaskDirection = 0.5 * Math.PI;
		} else {
			avoidingDirection = Parameters.Direction.LEFT;
			endTaskDirection = -0.5 * Math.PI;
		}
		counter = 0;
	}

	public void step() {
		
		// PLACEMENT DES ROBOTS ------------------------------
		//
		if (!startingMovesOver) {
			if (isHeading(endTaskDirection)) { startingMovesOver = true; move(); return; }
			stepTurn(avoidingDirection); return;
		}
		
		
		// IDENTIFICATION DES OBSTACLES ----------------------
		//
		obstacle = detectFront().getObjectType();
		if (obstacle == Types.WALL) avoidObstacle = true;
		
		
		// EVITER LES OBSTACLES ------------------------------
		//
		if (avoidObstacle && counter < 25) {
			if (counter < 25) { stepTurn(avoidingDirection); counter++; return; }
			else { avoidObstacle = false; return; }
		}
		
		
		// AVANCER -------------------------------------------
		//
		move();
		return;
	}
	
	
	// ---IDENTIFYING METHODS--- //

	private void identifyBots() {
		boolean teamB = true;
		whoAmI = BOTRADAR1;
		sendLogMessage("i am BOTRADAR1");
		for (IRadarResult o : detectRadar()) {
			if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) {
				whoAmI = BOTRADAR2;
				sendLogMessage("i am BOTRADAR2");
			}
			if (Math.abs(o.getObjectDirection()-Parameters.WEST) < Math.PI/4){
				teamB = false;
			}
		}
		if (teamB == false) {
			if (whoAmI == BOTRADAR1){
				whoAmI = BOTRADAR2;
				sendLogMessage("i am BOTRADAR2");
			} else {
				whoAmI = BOTRADAR1;
				sendLogMessage("i am BOTRADAR1");
			}
		}
	}
	
	// ---TOOLS--- //
	
	private boolean isHeading(double dir) {
		return Math.abs(Math.sin(getHeading() - dir)) < HEADINGPRECISION;
	}
	
	private boolean isSameDirection(double dir1, double dir2) {
		return Math.abs(dir1 - dir2) < ANGLEPRECISION;
	}
	
}
