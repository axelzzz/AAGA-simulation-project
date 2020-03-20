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

public class StrategyMainA extends Brain {
	// ---PARAMETERS---//
	private static final double ANGLEPRECISION = 0.01;
	private static final double FIREANGLEPRECISION = Math.PI / (double) 6;

	private static final int ALPHA = 0x1EADDA;
	private static final int BETA = 0x5EC0;
	private static final int GAMMA = 0x333;
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

	private static final int GAMMA_INIT_TASK = 11;
	private static final int BETA_INIT_TASK = 12;
	private static final int ALPHA_INIT_TASK = 13;
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

	// ---CONSTRUCTORS---//
	public StrategyMainA() {
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
			state = GAMMA_INIT_TASK;
		} else {
			myX = Parameters.teamAMainBot2InitX;
			myY = Parameters.teamAMainBot2InitY;
			state = BETA_INIT_TASK;
		}
		if (whoAmI == ALPHA) {
			myX = Parameters.teamAMainBot3InitX;
			myY = Parameters.teamAMainBot3InitY;
			state = ALPHA_INIT_TASK;
		}

		// INIT
		isMoving = false;
		fireOrder = false;
		fireRythm = 0;
		oldAngle = myGetHeading();
		targetX = 1500;
		targetY = 1000;
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
		if (debug && fireOrder)
			sendLogMessage("Firing enemy at (" + targetX + "," + targetY + ") !!");
		
		

		// COMMUNICATION
		ArrayList<String> messages = fetchAllMessages();
//		if (!messages.isEmpty())
//			System.out.println(messages);
		for (String m : messages)
			if (Integer.parseInt(m.split(":")[1]) == whoAmI || Integer.parseInt(m.split(":")[1]) == TEAM)
				process(m);

		// RADAR DETECTION
		freeze = false;
		friendlyFire = true;
		for (IRadarResult o : detectRadar()) {
			if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
					|| o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
				double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
				double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
				broadcast(whoAmI + ":" + TEAM + ":" + FIRE + ":" + enemyX + ":" + enemyY + ":" + OVER);
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
			firePosition(targetX, targetY);
			fireRythm++;
			return;
		}
		fireRythm++;
		if (fireRythm >= Parameters.bulletFiringLatency)
			fireRythm = 0;

		movingUpAndDown(); // TODO : traitement si toucher un debris/ennemi/ami

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
		if (Integer.parseInt(message.split(":")[2]) == FIRE) {
			fireOrder = true;
			countDown = 0;
			targetX = Double.parseDouble(message.split(":")[3]);
			targetY = Double.parseDouble(message.split(":")[4]);
		} else if (Integer.parseInt(message.split(":")[2]) == TURN_EAST_TASK) {
			TICK = 0;
			state = TURN_EAST_TASK;
		} else if (Integer.parseInt(message.split(":")[2]) == TURN_SOUTH_TASK) {
			state = TURN_SOUTH_TASK;
		} else if (Integer.parseInt(message.split(":")[2]) == TURN_NORTH_TASK) {
			state = TURN_NORTH_TASK;
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

	private void movingUpAndDown() {

		stretchUpAndDown(100);
		
		
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot) {
			sendLogMessage("BLOQUE VERS SUD PAR NOTRE ECLAIREUR");
			//state = TURN_EAST_TASK;
			fireOrder = false;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.TeamMainBot) {
			sendLogMessage("BLOQUE VERS SUD PAR NOTRE ATTAQUANT");
			//state = TURN_EAST_TASK;
			fireOrder = false;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot) {
			sendLogMessage("BLOQUE VERS NORD PAR NOTRE ECLAIREUR");
			//state = TURN_EAST_TASK;
			fireOrder = false;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.TeamMainBot) {
			sendLogMessage("BLOQUE VERS NORD PAR NOTRE ATTAQUANT");
			//state = TURN_EAST_TASK;
			fireOrder = false;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}		
		if (state == MOVE_EAST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot) {
			sendLogMessage("BLOQUE VERS EAST PAR NOTRE ECLAIREUR");
			//state = TURN_EAST_TASK;
			fireOrder = false;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_EAST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.TeamMainBot) {
			sendLogMessage("BLOQUE VERS EAST PAR NOTRE ATTAQUANT");
			//state = TURN_EAST_TASK;
			fireOrder = false;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		
		
		
		
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.OpponentSecondaryBot) {
			sendLogMessage("BLOQUE VERS SUD PAR LEUR ECLAIREUR");
			//state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.OpponentMainBot) {
			sendLogMessage("BLOQUE VERS SUD PAR LEUR ATTAQUANT");
			//state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.OpponentSecondaryBot) {
			sendLogMessage("BLOQUE VERS NORD PAR LEUR ECLAIREUR");
			//state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.OpponentMainBot) {
			sendLogMessage("BLOQUE VERS NORD PAR LEUR ATTAQUANT");
			//state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}		
		if (state == MOVE_EAST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.OpponentSecondaryBot) {
			sendLogMessage("BLOQUE VERS EAST PAR LEUR ECLAIREUR");
			//state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_EAST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.OpponentMainBot) {
			sendLogMessage("BLOQUE VERS EAST PAR LEUR ATTAQUANT");
			//state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		
		
		
		
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			sendLogMessage("BLOQUE VERS SUD PAR EPAVE");
			state = TURN_EAST_TASK;
			//(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			sendLogMessage("BLOQUE VERS SUD PAR EPAVE");
			state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			sendLogMessage("BLOQUE VERS NORD PAR EPAVE");
			state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			sendLogMessage("BLOQUE VERS NORD PAR EPAVE");
			state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_EAST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			sendLogMessage("BLOQUE VERS EAST PAR EPAVE");
			state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == MOVE_EAST_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			sendLogMessage("BLOQUE VERS EAST PAR EPAVE");
			state = TURN_EAST_TASK;
			//stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		
		
		if (state == TURN_SOUTH_TASK && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_SOUTH_TASK && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = MOVE_SOUTH_TASK;
			myMove();
			return;
		}
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
			myMove();
			return;
		}
		if (state == MOVE_SOUTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
			state = TURN_EAST_TASK;
			TICK = 0;
			broadcast(whoAmI + ":" + TEAM + ":" + TURN_EAST_TASK + ":" + OVER);
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		
		if (state == TURN_NORTH_TASK && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_NORTH_TASK && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = MOVE_NORTH_TASK;
			myMove();
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
			myMove();
			return;
		}
		if (state == MOVE_NORTH_TASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
			state = TURN_EAST_TASK;
			TICK = 0;
			broadcast(whoAmI + ":" + TEAM + ":" + TURN_EAST_TASK + ":" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		
		if (state == TURN_EAST_TASK && !(isSameDirection(getHeading(), Parameters.EAST))) {
			if (myY < 1000) {
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

		if (state == MOVE_EAST_TASK && TICK < 200) {
			myMove();
			return;
		}
		if (state == MOVE_EAST_TASK && TICK >= 200 && TICK < 210) {
			if (myY < 1000) {
				state = TURN_SOUTH_TASK;
			} else {
				state = TURN_NORTH_TASK;
			}
			broadcast(whoAmI + ":" + TEAM + ":" + state + ":" + OVER);
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		
		
		

	}

	private void stretchUpAndDown(int i) {
		if (state == GAMMA_INIT_TASK && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == GAMMA_INIT_TASK && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = MOVE_NORTH_TASK;
			myMove();
			return;
		}
		if (state == MOVE_NORTH_TASK && TICK < i) {
			myMove();
			return;
		}
		if (state == MOVE_NORTH_TASK && TICK >= i && TICK < i + 10) {
			state = TURN_SOUTH_TASK;
			stepTurn(Parameters.Direction.LEFT);
			return;
		}

		if (state == ALPHA_INIT_TASK && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == ALPHA_INIT_TASK && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = MOVE_SOUTH_TASK;
			myMove();
			return;
		}

		if (state == BETA_INIT_TASK && TICK >= i && TICK < i + 10) {
			state = TURN_SOUTH_TASK;
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
	}

}
