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

public class teamASecondary_v1 extends Brain {
	// ---PARAMETERS---//
	private static final double ANGLEPRECISION = 0.01;

	private static final int ROCKY = 0x1EADDB;
	private static final int MARIO = 0x5ED0;
	private static final int TEAM = 0xBADDAD;
	private static final int UNDEFINED = 0xBADC0DE0;

	private static final int FIRE = 0xB52;
	private static final int FALLBACK = 0xFA11BAC;
	private static final int ROGER = 0x0C0C0C0C;
	private static final int OVER = 0xC00010FF;
	private static final int COMM = 0xC00120F3;

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
	private ArrayList<Integer> positionList = new ArrayList<Integer>();


	// ---CONSTRUCTORS---//
	public teamASecondary_v1() {
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
		for (int i=0; i<10; i++) {
			positionList.add(0);
		}
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

		
		// COMMUNICATION
		broadcast(whoAmI + "/" + TEAM + "/" + COMM + "/" + state + "/" + myX + "/" + myY + "/" + OVER);
		ArrayList<String> messages = fetchAllMessages();
		for (String m : messages)
			if (Integer.parseInt(m.split("/")[1]) == whoAmI || Integer.parseInt(m.split("/")[1]) == TEAM)
				process(m);
		
		
		
		// RADAR DETECTION
		freeze = false;
		for (IRadarResult o : detectRadar()) {
			if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
					|| o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
				double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
				double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
				broadcast(whoAmI + "/" + TEAM + "/" + FIRE + "/" + enemyX + "/" + enemyY + "/" + OVER);
//				System.out.println(whoAmI + " find enemy : " + enemyX + "," + enemyY);
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
	
	
	
	
	
	
	
	
	
	
	private void process(String message) {
		if (Integer.parseInt(message.split("/")[2]) == 11111) {
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

			positionList.set(i, (int)Double.parseDouble(message.split("/")[3]));
			positionList.set(i+1, (int)Double.parseDouble(message.split("/")[4]));
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	
}
