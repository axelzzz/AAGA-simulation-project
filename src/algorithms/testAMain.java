/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/Stage1.java 2014-10-18 buixuan.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

import java.util.ArrayList;

public class testAMain extends Brain {
	// ---PARAMETERS---//
	private static final double ANGLEPRECISION = 0.01;
	private static final double FIREANGLEPRECISION = Math.PI / (double) 6;

	private static final int ALPHA = 0x1EADDA;
	private static final int BETA = 0x5EC0;
	private static final int GAMMA = 0x333;
	private static final int ROCKY = 0x1EADDB;
	private static final int MARIO = 0x5ED0;
	private static final int TEAM = 0xBADDAD;
	private static final int UNDEFINED = 0xBADC0DE0;

	private static final int FIRE = 0xB52;
	private static final int FALLBACK = 0xFA11BAC;
	private static final int ROGER = 0x0C0C0C0C;
	private static final int OVER = 0xC00010FF;

	private static final int TURN_SOUTH_TASK = 1;
	private static final int MOVE_SOUTH_TASK = 2;
	private static final int TURN_NORTH_TASK = 3;
	private static final int MOVE_NORTH_TASK = 4;
	private static final int TURN_EAST_TASK = 5;
	private static final int MOVE_EAST_TASK = 6;
	private static final int TURN_WEST_TASK = 7;
	private static final int MOVE_WEST_TASK = 8;

	private static final int GAMMA_TURN_TASK = 11;
	private static final int GAMMA_MOVE_TASK = 12;
	private static final int BETA_TURN_TASK = 13;
	private static final int ALPHA_TURN_TASK = 14;
	private static final int SINK = 0xBADC0DE1;

	// ---VARIABLES---//
	private int state;
	private double oldAngle;
	private double myX, myY;
	private boolean isMoving;
	private int whoAmI;
	private int fireRythm, rythm, counter;
	private int countDown;
	private double targetX, targetY;
	private boolean fireOrder;
	private boolean freeze;
	private boolean friendlyFire;
	private int TICK = 0;
	private boolean isMovingRight = true;
	private boolean isMovingDown = true;
	private ArrayList<Integer> positionList = new ArrayList<Integer>();

	// ---CONSTRUCTORS---//
	public testAMain() {
		super();
	}

	// ---ABSTRACT-METHODS-IMPLEMENTATION---//
	public void activate() {
		// ODOMETRY CODE
		whoAmI = GAMMA;
		for (IRadarResult o : detectRadar())
			if (isSameDirection(o.getObjectDirection(), Parameters.NORTH))
				whoAmI = ALPHA;
		for (IRadarResult o : detectRadar())
			if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH) && whoAmI != GAMMA)
				whoAmI = BETA;

		if (whoAmI == GAMMA) {
			myX = Parameters.teamAMainBot1InitX;
			myY = Parameters.teamAMainBot1InitY;
			state = GAMMA_TURN_TASK;
		} else {
			myX = Parameters.teamAMainBot2InitX;
			myY = Parameters.teamAMainBot2InitY;
			state = BETA_TURN_TASK;
		}
		if (whoAmI == ALPHA) {
			myX = Parameters.teamAMainBot3InitX;
			myY = Parameters.teamAMainBot3InitY;
			state = ALPHA_TURN_TASK;
//			state = SINK;
		}

		// INIT
		isMoving = false;
		fireOrder = false;
		fireRythm = 0;
		oldAngle = myGetHeading();
		targetX = 1500;
		targetY = 1000;
		for (int i = 0; i < 10; i++) {
			positionList.add(0);
		}
	}

	public void step() {
		
		
		if (TICK < 500)
			TICK++;

		// ODOMETRY CODE
		if (isMoving) {
			myX += Parameters.teamAMainBotSpeed * Math.cos(myGetHeading());
			myY += Parameters.teamAMainBotSpeed * Math.sin(myGetHeading());
			isMoving = false;
		}
		// DEBUG MESSAGE
		boolean debug = true;
		if (debug && whoAmI == ALPHA && state != SINK) {
			sendLogMessage("#ALPHA *thinks* (x,y)= (" + (int) myX + ", " + (int) myY + ") theta= "
					+ (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + state);
		}
		if (debug && whoAmI == BETA && state != SINK) {
			sendLogMessage("#BETA *thinks* (x,y)= (" + (int) myX + ", " + (int) myY + ") theta= "
					+ (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + state);
		}
		if (debug && whoAmI == GAMMA && state != SINK) {
			sendLogMessage("#GAMMA *thinks* (x,y)= (" + (int) myX + ", " + (int) myY + ") theta= "
					+ (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + state);
		}



//		if (debug && fireOrder)
//			sendLogMessage("Firing enemy at (" + targetX + "," + targetY + ") !!");

		// COMMUNICATION
		broadcast(whoAmI + "/" + TEAM + "/" + 11111 + "/" + myX + "/" + myY + "/" + OVER);
		ArrayList<String> messages = fetchAllMessages();
		for (String m : messages)
			if (Integer.parseInt(m.split("/")[1]) == whoAmI || Integer.parseInt(m.split("/")[1]) == TEAM)
				process(m);

		// RADAR DETECTION
		freeze = false;
		friendlyFire = true;
		for (IRadarResult o : detectRadar()) {
			if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
					|| o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
				double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
				double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
				broadcast(whoAmI + "/" + TEAM + "/" + FIRE + "/" + enemyX + "/" + enemyY + "/" + OVER);
			}
			if (o.getObjectDistance() <= 100 && !isRoughlySameDirection(o.getObjectDirection(), getHeading())
					&& o.getObjectType() != IRadarResult.Types.BULLET) {
				freeze = true;
			}
			if (o.getObjectType() == IRadarResult.Types.TeamMainBot
					|| o.getObjectType() == IRadarResult.Types.TeamSecondaryBot
					|| o.getObjectType() == IRadarResult.Types.Wreck) {
				if (fireOrder && onTheWay(o.getObjectDirection())) {
					friendlyFire = false;
				}
			}
		}
		if (freeze)
			return;

		// AUTOMATON
		if (fireOrder)
			countDown++;
		if (countDown >= 100)
			fireOrder = false;
		if (fireOrder && fireRythm == 0 && friendlyFire) {
			if (!isTeamInRange(targetX, targetY)) {
				firePosition(targetX, targetY);
			}
			
			fireRythm++;
			return;
		}
		fireRythm++;
		if (fireRythm >= Parameters.bulletFiringLatency)
			fireRythm = 0;


		turnBackwardIfExceed();

		patrol(); // La defense est la meilleure attaque

		if (state == FIRE) {
			if (fireRythm == 0) {
				firePosition(700, 1500);
				fireRythm++;
				return;
			}
			fireRythm++;
			if (fireRythm == Parameters.bulletFiringLatency)
				fireRythm = 0;
			if (rythm == 0)
				stepTurn(Parameters.Direction.LEFT);
			else
				myMove();
			rythm++;
			if (rythm == 14)
				rythm = 0;
			return;
		}

		if (state == SINK) {
			myMove();
			return;
		}
		if (true) {
			return;
		}
	}

	private void myMove() {
		isMoving = true;
		move();
	}

	private double myGetHeading() {
		return normalizeRadian(getHeading());
	}

	private double normalizeRadian(double angle) {
		double result = angle;
		while (result < 0)
			result += 2 * Math.PI;
		while (result >= 2 * Math.PI)
			result -= 2 * Math.PI;
		return result;
	}

	private boolean isSameDirection(double dir1, double dir2) {
		return Math.abs(normalizeRadian(dir1) - normalizeRadian(dir2)) < ANGLEPRECISION;
	}

	private boolean isRoughlySameDirection(double dir1, double dir2) {
		return Math.abs(normalizeRadian(dir1) - normalizeRadian(dir2)) < FIREANGLEPRECISION;
	}

	private void process(String message) {
		if (Integer.parseInt(message.split("/")[2]) == FIRE) {
			fireOrder = true;
			countDown = 0;
			targetX = Double.parseDouble(message.split("/")[3]);
			targetY = Double.parseDouble(message.split("/")[4]);
		} else if (Integer.parseInt(message.split("/")[2]) == TURN_SOUTH_TASK) {
			state = TURN_SOUTH_TASK;
			if (isMovingRight) {
				isMovingDown = true;
			} else {
				isMovingDown = false;
			}
		} else if (Integer.parseInt(message.split("/")[2]) == TURN_NORTH_TASK) {
			state = TURN_NORTH_TASK;
			if (isMovingRight) {
				isMovingDown = false;
			} else {
				isMovingDown = true;
			}
		} else if (Integer.parseInt(message.split("/")[2]) == TURN_EAST_TASK) {
			TICK = 0;
			isMovingDown = !isMovingDown;
			if (!isMovingRight) {
				isMovingRight = true;
			}
			state = TURN_EAST_TASK;

		} else if (Integer.parseInt(message.split("/")[2]) == TURN_WEST_TASK) {
			TICK = 0;
			isMovingDown = !isMovingDown;
			if (isMovingRight) {
				isMovingRight = false;
			}
			state = TURN_WEST_TASK;
		} else if (Integer.parseInt(message.split("/")[2]) == 11111) {
			int i = 0;
			if (Integer.parseInt(message.split("/")[0]) == 0x333) {
				i = 0;
			} else if (Integer.parseInt(message.split("/")[0]) == 0x5EC0) {
				i = 2;
			} else if (Integer.parseInt(message.split("/")[0]) == 0x1EADDA) {
				i = 4;
			} else if (Integer.parseInt(message.split("/")[0]) == 0x1EADDB) {
				i = 6;
			} else if (Integer.parseInt(message.split("/")[0]) == 0x5ED0) {
				i = 8;
			}

			positionList.set(i, (int) Double.parseDouble(message.split("/")[3]));
			positionList.set(i + 1, (int) Double.parseDouble(message.split("/")[4]));
		}

	}

	private void firePosition(double x, double y) {
		if (myX <= x)
			fire(Math.atan((y - myY) / (double) (x - myX)));
		else
			fire(Math.PI + Math.atan((y - myY) / (double) (x - myX)));
		return;
	}

	private boolean onTheWay(double angle) {
		if (myX <= targetX)
			return isRoughlySameDirection(angle, Math.atan((targetY - myY) / (double) (targetX - myX)));
		else
			return isRoughlySameDirection(angle, Math.PI + Math.atan((targetY - myY) / (double) (targetX - myX)));
	}

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	private void turnBackwardIfExceed() {
		if (myX<0) {
			state = TURN_EAST_TASK;
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (myX>3000) {
			state = TURN_WEST_TASK;
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (myY<0) {
			state = TURN_SOUTH_TASK;
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (myY>2000) {
			state = TURN_NORTH_TASK;
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
	}
	
	private void patrol() {

		stretchingUpAndDown(150);

		movingSouth();

		movingNorth();

		movingEast();

		movingWest();

	}

	private void stretchingUpAndDown(int i) {
		if (state == GAMMA_TURN_TASK && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == GAMMA_TURN_TASK && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = GAMMA_MOVE_TASK;
			myMove();
			return;
		}
		if (state == GAMMA_MOVE_TASK && TICK < i) {
			myMove();
			return;
		}
		if (state == GAMMA_MOVE_TASK && TICK >= i && TICK < i + 10) {
			state = TURN_SOUTH_TASK;
			stepTurn(Parameters.Direction.LEFT);
			return;
		}

		if (state == ALPHA_TURN_TASK && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == ALPHA_TURN_TASK && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = MOVE_SOUTH_TASK;
			myMove();
			return;
		}

		if (state == BETA_TURN_TASK && TICK >= i && TICK < i + 10) {
			state = TURN_SOUTH_TASK;
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
	}

	private boolean isObstacleAhead() {
		if (detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			return true;
		}
//		if (detectFront().getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot) {
//			return true;
//		}
//		if (detectFront().getObjectType() == IFrontSensorResult.Types.TeamMainBot) {
//			return true;
//		}
//		if (detectFront().getObjectType() == IFrontSensorResult.Types.OpponentSecondaryBot) {
//			return true;
//		}
//		if (detectFront().getObjectType() == IFrontSensorResult.Types.OpponentMainBot) {
//			return true;
//		}
		return false;
	}

	private void movingSouth() {
		if (state == TURN_SOUTH_TASK && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			if (isMovingRight) {
				stepTurn(Parameters.Direction.RIGHT);
			} else {
				stepTurn(Parameters.Direction.LEFT);
			}
			return;
		}
		if (state == TURN_SOUTH_TASK && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = MOVE_SOUTH_TASK;
			myMove();
			return;
		}
		if (state == MOVE_SOUTH_TASK && isObstacleAhead()) {
			if (isMovingRight) {
				state = TURN_EAST_TASK;
			} else {
				state = TURN_WEST_TASK;
			}
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == MOVE_SOUTH_TASK && (detectFront().getObjectType() != IFrontSensorResult.Types.WALL)) {
			myMove();
			return;
		}
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
			if (isMovingRight) {
				state = TURN_EAST_TASK;
			} else {
				state = TURN_WEST_TASK;
			}
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
	}

	private void movingNorth() {
		if (state == TURN_NORTH_TASK && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			if (isMovingRight) {
				stepTurn(Parameters.Direction.LEFT);
			} else {
				stepTurn(Parameters.Direction.RIGHT);
			}
			return;
		}
		if (state == TURN_NORTH_TASK && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = MOVE_NORTH_TASK;
			myMove();
			return;
		}
		if (state == MOVE_NORTH_TASK && isObstacleAhead()) {
			if (isMovingRight) {
				state = TURN_EAST_TASK;
			} else {
				state = TURN_WEST_TASK;
			}
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
			myMove();
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
			if (isMovingRight) {
				state = TURN_EAST_TASK;
			} else {
				state = TURN_WEST_TASK;
			}
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
	}

	private void movingEast() {
		if (state == TURN_EAST_TASK && !(isSameDirection(getHeading(), Parameters.EAST))) {
			if (isMovingDown) {
				stepTurn(Parameters.Direction.RIGHT);
			} else {
				stepTurn(Parameters.Direction.LEFT);
			}
			return;
		}
		if (state == TURN_EAST_TASK && isSameDirection(getHeading(), Parameters.EAST)) {
			state = MOVE_EAST_TASK;
			myMove();
			return;
		}
		if (state == MOVE_EAST_TASK && isObstacleAhead()) {
			if (isMovingDown) {
				state = TURN_SOUTH_TASK;
			} else {
				state = TURN_NORTH_TASK;
			}
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_EAST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
			state = TURN_WEST_TASK;
			isMovingRight = false;
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + TURN_WEST_TASK + "/" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_EAST_TASK && TICK < 300) {
			myMove();
			return;
		}
		if (state == MOVE_EAST_TASK && TICK >= 300 && TICK < 310) {
			if (isMovingDown) {
				state = TURN_SOUTH_TASK;
			} else {
				state = TURN_NORTH_TASK;
			}
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
	}

	private void movingWest() {
		if (state == TURN_WEST_TASK && !(isSameDirection(getHeading(), Parameters.WEST))) {
			if (isMovingDown) {
				stepTurn(Parameters.Direction.RIGHT);
			} else {
				stepTurn(Parameters.Direction.LEFT);
			}
			return;
		}
		if (state == TURN_WEST_TASK && isSameDirection(getHeading(), Parameters.WEST)) {
			state = MOVE_WEST_TASK;
			myMove();
			return;
		}
		if (state == MOVE_WEST_TASK && isObstacleAhead()) {
			if (isMovingDown) {
				state = TURN_NORTH_TASK;
			} else {
				state = TURN_SOUTH_TASK;
			}
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_WEST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
			state = TURN_EAST_TASK;
			isMovingRight = true;
			TICK = 0;
			broadcast(whoAmI + "/" + TEAM + "/" + TURN_EAST_TASK + "/" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_WEST_TASK && TICK < 300) {
			myMove();
			return;
		}
		if (state == MOVE_WEST_TASK && TICK >= 300 && TICK < 310) {
			if (isMovingDown) {
				state = TURN_NORTH_TASK;
			} else {
				state = TURN_SOUTH_TASK;
			}
			broadcast(whoAmI + "/" + TEAM + "/" + state + "/" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
	}

	private boolean isTeamInRange(double x, double y){
		double direction = 0;
		if (myX <= x) {
			direction = (Math.atan((y - myY) / (double) (x - myX)));
		} else {
			direction = (Math.PI + Math.atan((y - myY) / (double) (x - myX)));
		}
		
		for (int i=0; i<positionList.size(); i=i+2) {			
			if (myX <= positionList.get(i)) {
				if (isSameDirection(direction, Math.atan((positionList.get(i+1) - myY) / (double) (positionList.get(i) - myX)))) {
					return true;
				}
			} else {
				if (isSameDirection(direction, Math.PI + Math.atan((positionList.get(i+1) - myY) / (double) (positionList.get(i) - myX)))) {
					return true;
				}
			}
		}
		
		return false;
	}
	
	
	
	
	
	
	
}