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

public class StrategySecondaryA extends Brain {
	// ---PARAMETERS---//
	private static final double ANGLEPRECISION = 0.01;

	private static final int ROCKY = 0x1EADDA;
	private static final int MARIO = 0x5EC0;
	private static final int TEAM = 0xBADDAD;
	private static final int UNDEFINED = 0xBADC0DE0;

	private static final int FIRE = 0xB52;
	private static final int FALLBACK = 0xFA11BAC;
	private static final int ROGER = 0x0C0C0C0C;
	private static final int OVER = 0xC00010FF;

	private static final int TURNLEFTTASKROCKY = 1;
	private static final int MOVETASKROCKY = 2;
	private static final int TURNRIGHTTASKROCKY = 3;
	
	private static final int TURNLEFTTASKMARIO = 1;
	private static final int MOVETASKMARIO = 2;
	private static final int TURNRIGHTTASKMARIO = 3;
	
	private static final int SINK = 0xBADC0DE1;

	// ---VARIABLES---//
	private int state;
	private double oldAngle;
	private double myX, myY;
	private boolean isMoving;
	private boolean freeze;
	private int whoAmI;

	// ---CONSTRUCTORS---//
	public StrategySecondaryA() {
		super();
	}

	// ---ABSTRACT-METHODS-IMPLEMENTATION---//
	public void activate() {
		// ODOMETRY CODE
		whoAmI = ROCKY;
		for (IRadarResult o : detectRadar())
			if (isSameDirection(o.getObjectDirection(), Parameters.NORTH))
				whoAmI = MARIO;
		
		if (whoAmI == ROCKY) {
			myX = Parameters.teamASecondaryBot1InitX;
			myY = Parameters.teamASecondaryBot1InitY;
		} else {
			myX = Parameters.teamASecondaryBot2InitX;
			myY = Parameters.teamASecondaryBot2InitY;
		}

		// INIT
		state = TURNLEFTTASKROCKY;
		isMoving = false;
		oldAngle = getHeading();
	}

	public void step() {
		// ODOMETRY CODE
		if (isMoving) {
			myX += Parameters.teamASecondaryBotSpeed * Math.cos(getHeading());
			myY += Parameters.teamASecondaryBotSpeed * Math.sin(getHeading());
			isMoving = false;
		}
		// DEBUG MESSAGE
		if (whoAmI == ROCKY)
			sendLogMessage("#ROCKY *thinks* he is rolling at position (" + (int) myX + ", " + (int) myY + ").");
		else
			sendLogMessage("#MARIO *thinks* he is rolling at position (" + (int) myX + ", " + (int) myY + ").");

		// RADAR DETECTION
		freeze = false;
		for (IRadarResult o : detectRadar()) {
			if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
					|| o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
				double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
				double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
				broadcast(whoAmI + ":" + TEAM + ":" + FIRE + ":" + enemyX + ":" + enemyY + ":" + OVER);
			}
			if (o.getObjectDistance() <= 100) {
				freeze = true;
			}
		}
		if (freeze)
			return;

		// AUTOMATON
		
		
		if (state == TURNLEFTTASKROCKY && !(isSameDirection(getHeading(), Parameters.NORTH)) && whoAmI == ROCKY) {
			stepTurn(Parameters.Direction.LEFT);
			// sendLogMessage("Initial TeamA Secondary Bot1 position. Heading North!");
			return;
		}
		if (state == TURNLEFTTASKROCKY && isSameDirection(getHeading(), Parameters.NORTH) && whoAmI == ROCKY) {
			state = MOVETASKROCKY;
			myMove();
			// sendLogMessage("Moving a head. Waza!");
			return;
		}
		if (state == MOVETASKROCKY && detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING && whoAmI == ROCKY) {
			myMove(); // And what to do when blind blocked?
			// sendLogMessage("Moving a head. Waza!");
			return;
		}
		
		if (state == MOVETASKROCKY && detectFront().getObjectType() != IFrontSensorResult.Types.BULLET && whoAmI == ROCKY) {
			state = TURNRIGHTTASKROCKY;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		
		if (state == MOVETASKROCKY && detectFront().getObjectType() != IFrontSensorResult.Types.Wreck && whoAmI == ROCKY) {
			state = TURNRIGHTTASKROCKY;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		
		if (state == MOVETASKROCKY && detectFront().getObjectType() != IFrontSensorResult.Types.TeamMainBot && whoAmI == ROCKY) {
			state = TURNRIGHTTASKROCKY;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		
		if (state == MOVETASKROCKY && detectFront().getObjectType() != IFrontSensorResult.Types.NOTHING && whoAmI == ROCKY) {
			state = TURNRIGHTTASKROCKY;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		if (state == TURNRIGHTTASKROCKY && !(isSameDirection(getHeading(), oldAngle + Parameters.RIGHTTURNFULLANGLE)) && whoAmI == ROCKY) {
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		if (state == TURNRIGHTTASKROCKY && isSameDirection(getHeading(), oldAngle + Parameters.RIGHTTURNFULLANGLE) && whoAmI == ROCKY) {
			state = MOVETASKROCKY;
			myMove();
			// sendLogMessage("Moving a head. Waza!");
			return;
		}
		
		
		if (state == TURNLEFTTASKMARIO && !(isSameDirection(getHeading(), Parameters.NORTH)) && whoAmI == MARIO) {
			stepTurn(Parameters.Direction.LEFT);
			// sendLogMessage("Initial TeamA Secondary Bot1 position. Heading North!");
			return;
		}
		if (state == TURNLEFTTASKMARIO && isSameDirection(getHeading(), Parameters.NORTH) && whoAmI == MARIO) {
			state = MOVETASKMARIO;
			myMove();
			// sendLogMessage("Moving a head. Waza!");
			return;
		}
		if (state == MOVETASKMARIO && detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING && whoAmI == MARIO) {
			myMove(); // And what to do when blind blocked?
			// sendLogMessage("Moving a head. Waza!");
			return;
		}
		
		if (state == MOVETASKMARIO && detectFront().getObjectType() != IFrontSensorResult.Types.BULLET && whoAmI == MARIO) {
			state = TURNRIGHTTASKMARIO;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		
		if (state == MOVETASKMARIO && detectFront().getObjectType() != IFrontSensorResult.Types.Wreck && whoAmI == MARIO) {
			state = TURNRIGHTTASKMARIO;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		
		if (state == MOVETASKMARIO && detectFront().getObjectType() != IFrontSensorResult.Types.TeamMainBot && whoAmI == MARIO) {
			state = TURNRIGHTTASKMARIO;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		
		if (state == MOVETASKMARIO && detectFront().getObjectType() != IFrontSensorResult.Types.NOTHING && whoAmI == MARIO) {
			state = TURNRIGHTTASKMARIO;
			oldAngle = getHeading();
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		if (state == TURNRIGHTTASKMARIO && !(isSameDirection(getHeading(), oldAngle + Parameters.RIGHTTURNFULLANGLE)) && whoAmI == MARIO) {
			stepTurn(Parameters.Direction.RIGHT);
			// sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
			return;
		}
		if (state == TURNRIGHTTASKMARIO && isSameDirection(getHeading(), oldAngle + Parameters.RIGHTTURNFULLANGLE) && whoAmI == MARIO) {
			state = MOVETASKMARIO;
			myMove();
			// sendLogMessage("Moving a head. Waza!");
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

	private boolean isSameDirection(double dir1, double dir2) {
		return Math.abs(dir1 - dir2) < ANGLEPRECISION;
	}
}
