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

public class teamBMain extends Brain {
	// ---PARAMETERS---//
	private static final double ANGLEPRECISION = 0.01;
	private static final double FIREANGLEPRECISION = Math.PI / (double) 6;

	private static final int ALPHA = 0x1EADDA;
	private static final int BETA = 0x5EC0;
	private static final int GAMMA = 0x333;
	private static final int ROCKY = 0x1EADDB;
	private static final int MARIO = 0x5ED0;
	private static final int TEAM = 0xBAD1AD;
	private static final int UNDEFINED = 0xBADC0DE0;

	private static final int FIRE = 0xB52;
	private static final int FALLBACK = 0xFA11BAC;
	private static final int ROGER = 0x0C0C0C0C;
	private static final int OVER = 0xC00010FF;
	private static final int COMM = 0xC00120F3;
	private static final int FINDBULLET = 0xC00220F3;
	private static final int WRECK = 0x220C5C0C;
	private static final int OpponentMainBot = 0x21035C4;

	private static final int TURN_SOUTH_TASK = 1;
	private static final int MOVE_SOUTH_TASK = 2;
	private static final int TURN_NORTH_TASK = 3;
	private static final int MOVE_NORTH_TASK = 4;
	private static final int TURN_EAST_TASK = 5;
	private static final int MOVE_EAST_TASK = 6;
	private static final int TURN_WEST_TASK = 7;
	private static final int MOVE_WEST_TASK = 8;

	private static final int TURN_INIT_TASK = 11;
	private static final int MOVE_INIT_TASK = 12;

	private static final int FIRE_TASK = 21;
	private static final int FIRE_BACK_TASK = 22;
	private static final int MOVE_TASK = 23;
	private static final int MOVE_BACK_TASK = 24;

	private static final int END_OF_FE_TASK_1 = 31;
	private static final int END_OF_FE_TASK_2 = 32;
	private static final int END_OF_FE_TASK_3 = 33;
	private static final int END_OF_FE_TASK_4 = 34;
	private static final int END_OF_FE_TASK_5 = 35;
	private static final int END_OF_FE_TASK_6 = 36;
	private static final int END_OF_FE_TASK_7 = 37;
	private static final int END_OF_FE_TASK_8 = 38;
	private static final int END_OF_FE_TASK_9 = 39;
	private static final int END_OF_FE_TASK_0 = 40;

	private static final int NOT_FIFTH_ELEMENT = 41;

	private static final int BACK_TO_STARTING_POINT_TASK = 51;
	private static final int MOVE_AND_FIRE_TASK = 52;
	private static final int MOVE_BACK_AND_FIRE_TASK = 53;

	private static final int TURN_NORTH_TO_BYBASS_EAST_TASK_1 = 61;
	private static final int TURN_NORTH_TO_BYBASS_EAST_TASK_2 = 62;
	private static final int TURN_NORTH_TO_BYBASS_EAST_TASK_3 = 63;
	private static final int TURN_NORTH_TO_BYBASS_EAST_TASK_4 = 64;
	private static final int TURN_NORTH_TO_BYBASS_EAST_TASK_5 = 65;
	private static final int TURN_NORTH_TO_BYBASS_EAST_TASK_6 = 66;
	private static final int TURN_NORTH_TO_BYBASS_EAST_TASK_7 = 67;

	private static final int TURN_SOUTH_TO_BYBASS_EAST_TASK_1 = 71;
	private static final int TURN_SOUTH_TO_BYBASS_EAST_TASK_2 = 72;
	private static final int TURN_SOUTH_TO_BYBASS_EAST_TASK_3 = 73;
	private static final int TURN_SOUTH_TO_BYBASS_EAST_TASK_4 = 74;
	private static final int TURN_SOUTH_TO_BYBASS_EAST_TASK_5 = 75;
	private static final int TURN_SOUTH_TO_BYBASS_EAST_TASK_6 = 76;
	private static final int TURN_SOUTH_TO_BYBASS_EAST_TASK_7 = 77;

	private static final int TURN_NORTH_TO_BYBASS_WEST_TASK_1 = 81;
	private static final int TURN_NORTH_TO_BYBASS_WEST_TASK_2 = 82;
	private static final int TURN_NORTH_TO_BYBASS_WEST_TASK_3 = 83;
	private static final int TURN_NORTH_TO_BYBASS_WEST_TASK_4 = 84;
	private static final int TURN_NORTH_TO_BYBASS_WEST_TASK_5 = 85;
	private static final int TURN_NORTH_TO_BYBASS_WEST_TASK_6 = 86;
	private static final int TURN_NORTH_TO_BYBASS_WEST_TASK_7 = 87;

	private static final int TURN_SOUTH_TO_BYBASS_WEST_TASK_1 = 91;
	private static final int TURN_SOUTH_TO_BYBASS_WEST_TASK_2 = 92;
	private static final int TURN_SOUTH_TO_BYBASS_WEST_TASK_3 = 93;
	private static final int TURN_SOUTH_TO_BYBASS_WEST_TASK_4 = 94;
	private static final int TURN_SOUTH_TO_BYBASS_WEST_TASK_5 = 95;
	private static final int TURN_SOUTH_TO_BYBASS_WEST_TASK_6 = 96;
	private static final int TURN_SOUTH_TO_BYBASS_WEST_TASK_7 = 97;

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
	private ArrayList<ArrayList<Integer>> memberList = new ArrayList<ArrayList<Integer>>();
	private boolean detectBullet;
	private double bulletDirection;
	private boolean wreckG = false;
	private boolean wreckB = false;
	private boolean wreckA = false;
	private boolean wreckALL = false;
	private boolean OMBG = false;
	private boolean OMBB = false;
	private boolean OMBA = false;
	private boolean OMBALL = false;
	private boolean Is_Fifth_Element = true;
	private boolean Is_Move_Fire = false;
	private int oldX, oldY;

	// ---CONSTRUCTORS---//
	public teamBMain() {
		super();
	}

	// ---ABSTRACT-METHODS-IMPLEMENTATION---//
	public void activate() {

		logIn();

	}

	public void step() {

		displayMessage();

		processMessages();

		// RADAR DETECTION
		freeze = false;
		friendlyFire = true;
		for (IRadarResult o : detectRadar()) {
			if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
					|| o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
				double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
				double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
				broadcast(whoAmI + "/" + TEAM + "/" + FIRE + "/" + enemyX + "/" + enemyY + "/" + OVER);
//				System.out.println(whoAmI + "/" + TEAM + "/" + FIRE + "/" + enemyX + "/" + enemyY + "/" + myX);
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

			if (!Is_Fifth_Element) {
				if (o.getObjectType() == IRadarResult.Types.Wreck) {
					int wreckX = (int) (myX + o.getObjectDistance() * Math.cos(o.getObjectDirection()));
					int wreckY = (int) (myY + o.getObjectDistance() * Math.sin(o.getObjectDirection()));
					int dist = distance(myX, myY, wreckX, wreckY);
					if (state == MOVE_AND_FIRE_TASK) {
						if (dist < 150 && wreckY >= myY - 100 && wreckY <= myY + 100 && wreckX<=myX) {
							if (wreckY >= myY) {
//								state = TURN_NORTH_TO_BYBASS_EAST_TASK_1;
								state = TURN_NORTH_TO_BYBASS_WEST_TASK_1;
							} else {
//								state = TURN_SOUTH_TO_BYBASS_EAST_TASK_1;
								state = TURN_SOUTH_TO_BYBASS_WEST_TASK_1;
							}
						}
					} else if (state == MOVE_BACK_AND_FIRE_TASK) {
						if (dist < 150 && wreckY >= myY - 100 && wreckY <= myY + 100 && wreckX>myX) {
							if (wreckY >= myY) {
//								state = TURN_NORTH_TO_BYBASS_WEST_TASK_1;
								state = TURN_NORTH_TO_BYBASS_EAST_TASK_1;
							} else {
//								state = TURN_SOUTH_TO_BYBASS_WEST_TASK_1;
								state = TURN_SOUTH_TO_BYBASS_EAST_TASK_1;
							}
						}
					}

				}
			}
		}
		if (freeze)
			return;

		
		
		
		
		if (Is_Fifth_Element) {
			cleanUpMiddle();
		} else {
			CounterStrike();
		}
		
		
		
		

		if (true) {
			return;
		}
	}

	private void myMove() {
		move();
		myX += Parameters.teamAMainBotSpeed * Math.cos(myGetHeading());
		myY += Parameters.teamAMainBotSpeed * Math.sin(myGetHeading());
		broadcast(whoAmI + "/" + TEAM + "/" + COMM + "/" + state + "/" + myX + "/" + myY + "/" + OVER);
	}

	
	private void myMoveBack() {
		moveBack();
		myX -= Parameters.teamAMainBotSpeed * Math.cos(myGetHeading());
		myY -= Parameters.teamAMainBotSpeed * Math.sin(myGetHeading());
		broadcast(whoAmI + "/" + TEAM + "/" + COMM + "/" + state + "/" + myX + "/" + myY + "/" + OVER);
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
		return (Math.abs(normalizeRadian(dir1) - normalizeRadian(dir2))) < ANGLEPRECISION;
	}

	
	private boolean isRoughlySameDirection(double dir1, double dir2) {

		return normalizeRadian(Math.abs((dir1) - (dir2))) < FIREANGLEPRECISION;
	}

	
	private void firePosition(double x, double y) {
		if (myX <= x) {
			if (!isTeamInRange(Math.atan((y - myY) / (double) (x - myX)))) {
				fire(Math.atan((y - myY) / (double) (x - myX)));
			}
//			fire(Math.atan((y - myY) / (double) (x - myX)));
		} else {
			if (!isTeamInRange(Math.PI + Math.atan((y - myY) / (double) (x - myX)))) {
				fire(Math.PI + Math.atan((y - myY) / (double) (x - myX)));
			}
//			fire(Math.PI + Math.atan((y - myY) / (double) (x - myX)));
		}
		return;
	}

	
	private boolean onTheWay(double angle) {
		if (myX <= targetX)
			return isRoughlySameDirection(angle, Math.atan((targetY - myY) / (double) (targetX - myX)));
		else
			return isRoughlySameDirection(angle, Math.PI + Math.atan((targetY - myY) / (double) (targetX - myX)));
	}

	
	private boolean onTheWay(double angle, double x, double y) {
		if (myX <= x)
			return isRoughlySameDirection(angle, Math.atan((y - myY) / (double) (x - myX)));
		else
			return isRoughlySameDirection(angle, Math.PI + Math.atan((y - myY) / (double) (x - myX)));
	}

	
	
	private void logIn() {
		whoAmI = GAMMA;
		for (IRadarResult o : detectRadar())
			if (isSameDirection(o.getObjectDirection(), Parameters.NORTH))
				whoAmI = ALPHA;
		for (IRadarResult o : detectRadar())
			if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH) && whoAmI != GAMMA)
				whoAmI = BETA;

		if (whoAmI == GAMMA) {
			myX = Parameters.teamBMainBot1InitX;
			myY = Parameters.teamBMainBot1InitY;
		} else {
			myX = Parameters.teamBMainBot2InitX;
			myY = Parameters.teamBMainBot2InitY;
		}
		if (whoAmI == ALPHA) {
			myX = Parameters.teamBMainBot3InitX;
			myY = Parameters.teamBMainBot3InitY;
		}

		// INIT
		state = MOVE_WEST_TASK;
		isMoving = false;
		fireOrder = false;
		fireRythm = 0;
		oldAngle = myGetHeading();
		targetX = 1500;
		targetY = 1000;
		for (int i = 0; i < 5; i++) {
			ArrayList<Integer> tmp = new ArrayList<Integer>();
			for (int j = 0; j < 4; j++) {
				tmp.add(0);
			}
			memberList.add(tmp);
		}
		detectBullet = true;
	}

		
	private void displayMessage() {
//		if (whoAmI == ALPHA) {
//			sendLogMessage("#ALPHA : (x,y)= (" + (int) myX + ", " + (int) myY + ") theta= "
//					+ (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + state);
//		}
//		if (whoAmI == BETA) {
//			sendLogMessage("#BETA : (x,y)= (" + (int) myX + ", " + (int) myY + ") theta= "
//					+ (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + state);
//		}
//		if (whoAmI == GAMMA) {
//			sendLogMessage("#GAMMA : (x,y)= (" + (int) myX + ", " + (int) myY + ") theta= "
//					+ (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + state);
//		}

		if (whoAmI == ALPHA) {
			sendLogMessage("#A : (" + (int) myX + "," + (int) myY + ") State= " + state 
					+ " (" + (int)memberList.get(2).get(2)+","+(int)memberList.get(2).get(3) + ")" + " " + detectFront().getObjectType());
		}
		if (whoAmI == BETA) {
			sendLogMessage("#B : (" + (int) myX + "," + (int) myY + ") State= " + state 
					+ " (" + (int)memberList.get(1).get(2)+","+(int)memberList.get(1).get(3) + ")" + " " + detectFront().getObjectType());
		}
		if (whoAmI == GAMMA) {
			sendLogMessage("#G : (" + (int) myX + "," + (int) myY + ") State= " + state 
					+ " (" + (int)memberList.get(0).get(2)+","+(int)memberList.get(0).get(3) + ")" + " " + detectFront().getObjectType());
		}

//		if (fireOrder) {
//			sendLogMessage("Firing enemy at ("+ (int)targetX+","+(int)targetY+")" );
//		}
	}

		
	private void processMessages() {
		ArrayList<String> messages = fetchAllMessages();
		for (String m : messages)
			if (Integer.parseInt(m.split("/")[1]) == TEAM)
				process(m);
	}

		
	private void process(String message) {
		if (Integer.parseInt(message.split("/")[2]) == FIRE) {
			fireOrder = true;
			countDown = 0;
			targetX = Double.parseDouble(message.split("/")[3]);
			targetY = Double.parseDouble(message.split("/")[4]);
		} else if (Integer.parseInt(message.split("/")[2]) == COMM) {
			int i = -1;
			if (Integer.parseInt(message.split("/")[0]) == GAMMA) {
				i = 0;
			} else if (Integer.parseInt(message.split("/")[0]) == BETA) {
				i = 1;
			} else if (Integer.parseInt(message.split("/")[0]) == ALPHA) {
				i = 2;
			} else if (Integer.parseInt(message.split("/")[0]) == ROCKY) {
				i = 3;
			} else if (Integer.parseInt(message.split("/")[0]) == MARIO) {
				i = 4;
			}
			memberList.get(i).set(0, (int) Double.parseDouble(message.split("/")[0]));
			memberList.get(i).set(1, (int) Double.parseDouble(message.split("/")[3]));
			memberList.get(i).set(2, (int) Double.parseDouble(message.split("/")[4]));
			memberList.get(i).set(3, (int) Double.parseDouble(message.split("/")[5]));
		} else if (Integer.parseInt(message.split("/")[2]) == WRECK) {
			if (Integer.parseInt(message.split("/")[0]) == GAMMA) {
				wreckG = true;
			} else if (Integer.parseInt(message.split("/")[0]) == BETA) {
				wreckB = true;
			} else if (Integer.parseInt(message.split("/")[0]) == ALPHA) {
				wreckA = true;
			}
		} else if (Integer.parseInt(message.split("/")[2]) == OpponentMainBot) {
			if (Integer.parseInt(message.split("/")[0]) == GAMMA) {
				OMBG = true;
			} else if (Integer.parseInt(message.split("/")[0]) == BETA) {
				OMBB = true;
			} else if (Integer.parseInt(message.split("/")[0]) == ALPHA) {
				OMBA = true;
			}
		} else if (Integer.parseInt(message.split("/")[2]) == END_OF_FE_TASK_1) {
			state = END_OF_FE_TASK_1;
			wreckALL = true;
		} else if (Integer.parseInt(message.split("/")[2]) == NOT_FIFTH_ELEMENT) {
//			System.out.println("\n\n\n M : get NOT_FIFTH_ELEMENT => Is_Fifth_Element=false \n\n\n");
			Is_Fifth_Element = false;
//			state = BACK_TO_STARTING_POINT_TASK;
			state = MOVE_AND_FIRE_TASK;
		} else if (Integer.parseInt(message.split("/")[2]) == MOVE_AND_FIRE_TASK) {
			state = MOVE_AND_FIRE_TASK;
		} else if (Integer.parseInt(message.split("/")[2]) == MOVE_BACK_AND_FIRE_TASK) {
			state = MOVE_BACK_AND_FIRE_TASK;
		}
	}
	

	private int distance(double x, double y, double x1, double y1) {
		return (int) Math.sqrt(Math.abs((x - x1) * (x - x1) + (y - y1) * (y - y1)));
	}

		
	private boolean isTeamInRange(double angle) {
		for (int i = 0; i < memberList.size(); i++) {
			if (whoAmI != memberList.get(i).get(0)
					&& distance(myX, myY, memberList.get(i).get(2), memberList.get(i).get(3)) < Parameters.bulletRange
					&& onTheWay(angle, memberList.get(i).get(2), memberList.get(i).get(3))) {
				return true;
			}
		}
		return false;
	}

		
	private void cleanUpMiddle() {

		if (wreckG && wreckB && wreckA && Is_Fifth_Element && !wreckALL) {
			broadcast(whoAmI + "/" + TEAM + "/" + END_OF_FE_TASK_1 + "/" + OVER);
		}
		if (detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			if (whoAmI == GAMMA && !wreckG) {
				broadcast(whoAmI + "/" + TEAM + "/" + WRECK + "/" + OVER);
			}
			if (whoAmI == BETA && !wreckB) {
				broadcast(whoAmI + "/" + TEAM + "/" + WRECK + "/" + OVER);
			}
			if (whoAmI == ALPHA && !wreckA) {
				broadcast(whoAmI + "/" + TEAM + "/" + WRECK + "/" + OVER);
			}
		}

		if (myX > 2020 && myX < 2025 && !(OMBG && OMBB && OMBA)) {
//			System.out.println("---------- myx="+myX+" OMBG="+OMBG+" OMBB="+OMBB+" OMBA="+OMBA );
			broadcast(whoAmI + "/" + TEAM + "/" + NOT_FIFTH_ELEMENT + "/" + OVER);
		}
		if (detectFront().getObjectType() == IFrontSensorResult.Types.OpponentMainBot) {
			if (whoAmI == GAMMA && !OMBG) {
				broadcast(whoAmI + "/" + TEAM + "/" + OpponentMainBot + "/" + OVER);
			}
			if (whoAmI == BETA && !OMBB) {
				broadcast(whoAmI + "/" + TEAM + "/" + OpponentMainBot + "/" + OVER);
			}
			if (whoAmI == ALPHA && !OMBA) {
				broadcast(whoAmI + "/" + TEAM + "/" + OpponentMainBot + "/" + OVER);
			}
		}

		if (state == FIRE_TASK) {
			state = MOVE_WEST_TASK;
			if (!isTeamInRange(Math.PI)) {
				fire(Math.PI);
			}
			return;
		}
		if (state == MOVE_WEST_TASK) {
			state = FIRE_TASK;
			myMove();
			return;
		}

		if (state == END_OF_FE_TASK_1 && detectFront().getObjectType() == IFrontSensorResult.Types.Wreck) {
			myMoveBack();
			return;
		}
		if (state == END_OF_FE_TASK_1 && detectFront().getObjectType() != IFrontSensorResult.Types.Wreck) {
			state = END_OF_FE_TASK_2;
			return;
		}
		if (whoAmI == GAMMA && state == END_OF_FE_TASK_2) {
			state = END_OF_FE_TASK_3;
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (whoAmI == GAMMA && state == END_OF_FE_TASK_3 && !isSameDirection(getHeading(), Parameters.NORTH)) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (whoAmI == GAMMA && state == END_OF_FE_TASK_3 && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = END_OF_FE_TASK_4;
			myMove();
			return;
		}
		if (whoAmI == GAMMA && state == END_OF_FE_TASK_4 && myY > 10) {
			myMove();
			return;
		}
		if (whoAmI == GAMMA && state == END_OF_FE_TASK_4 && myY <= 10) {
			state = END_OF_FE_TASK_5;
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (whoAmI == GAMMA && state == END_OF_FE_TASK_5 && !isSameDirection(getHeading(), Parameters.WEST)) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (whoAmI == GAMMA && state == END_OF_FE_TASK_5 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = END_OF_FE_TASK_6;
			myMove();
			return;
		}
		if (state == END_OF_FE_TASK_6) {
			state = END_OF_FE_TASK_7;
			fire(Math.PI);
			return;
		}
		if (state == END_OF_FE_TASK_7) {
			state = END_OF_FE_TASK_6;
			myMove();
			return;
		}
		if (whoAmI == ALPHA && state == END_OF_FE_TASK_2) {
			state = END_OF_FE_TASK_3;
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (whoAmI == ALPHA && state == END_OF_FE_TASK_3 && !isSameDirection(getHeading(), Parameters.SOUTH)) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (whoAmI == ALPHA && state == END_OF_FE_TASK_3 && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = END_OF_FE_TASK_4;
			myMove();
			return;
		}
		if (whoAmI == ALPHA && state == END_OF_FE_TASK_4 && myY < 1990) {
			myMove();
			return;
		}
		if (whoAmI == ALPHA && state == END_OF_FE_TASK_4 && myY >= 1990) {
			state = END_OF_FE_TASK_5;
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (whoAmI == ALPHA && state == END_OF_FE_TASK_5 && !isSameDirection(getHeading(), Parameters.WEST)) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (whoAmI == ALPHA && state == END_OF_FE_TASK_5 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = END_OF_FE_TASK_6;
			myMove();
			return;
		}
		if (state == END_OF_FE_TASK_6) {
			state = END_OF_FE_TASK_7;
			fire(Math.PI);
			return;
		}
		if (state == END_OF_FE_TASK_7) {
			state = END_OF_FE_TASK_6;
			myMove();
			return;
		}

	}

	
	
	private void CounterStrike() {

		if (myX >= 2850)
			broadcast(whoAmI + "/" + TEAM + "/" + MOVE_AND_FIRE_TASK + "/" + OVER);
		if (myX <= 150)
			broadcast(whoAmI + "/" + TEAM + "/" + MOVE_BACK_AND_FIRE_TASK + "/" + OVER);

		bypassWreck();

		if (state == MOVE_AND_FIRE_TASK)
			moveAndFire();
		if (state == MOVE_BACK_AND_FIRE_TASK)
			moveBackAndFire();

	}
		
	private void moveAndFire() {
		if (fireOrder) {
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
		} else {
			if (Is_Move_Fire) {
				Is_Move_Fire = false;
				if (!isTeamInRange(Math.PI)) {
					fire(Math.PI);
				}
				return;
			} else {
				Is_Move_Fire = true;
				myMove();
				return;
			}
		}
		return;
	}
		
	private void moveBackAndFire() {
		if (fireOrder) {
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
		} else {
			if (Is_Move_Fire) {
				Is_Move_Fire = false;
				if (!isTeamInRange(0)) {
					fire(0);
				}
				return;
			} else {
				Is_Move_Fire = true;
				myMoveBack();
				return;
			}
		}
		return;
	}

	private void bypassWreck() {
		bypassWreck_N_E();
		bypassWreck_S_E();
		bypassWreck_N_W();
		bypassWreck_S_W();
	}
		
	private void bypassWreck_N_E() {

		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_1 && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_1 && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = TURN_NORTH_TO_BYBASS_EAST_TASK_2;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_2 && oldY - myY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_2 && oldY - myY > 100) {
			stepTurn(Parameters.Direction.RIGHT);
			state = TURN_NORTH_TO_BYBASS_EAST_TASK_3;
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_3 && !(isSameDirection(getHeading(), Parameters.EAST))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_3 && isSameDirection(getHeading(), Parameters.EAST)) {
			state = TURN_NORTH_TO_BYBASS_EAST_TASK_4;
			oldX = (int) myX;
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_4 && myX - oldX <= 350) {
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_4 && myX - oldX > 350) {
			stepTurn(Parameters.Direction.RIGHT);
			state = TURN_NORTH_TO_BYBASS_EAST_TASK_5;
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_5 && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_5 && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = TURN_NORTH_TO_BYBASS_EAST_TASK_6;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_6 && myY - oldY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_6 && myY - oldY > 100) {
			stepTurn(Parameters.Direction.RIGHT);
			state = TURN_NORTH_TO_BYBASS_EAST_TASK_7;
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_7 && !(isSameDirection(getHeading(), Parameters.WEST))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_EAST_TASK_7 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = MOVE_BACK_AND_FIRE_TASK;
			return;
		}

	}

	private void bypassWreck_S_E() {

		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_1 && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_1 && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = TURN_SOUTH_TO_BYBASS_EAST_TASK_2;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_2 && myY - oldY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_2 && myY - oldY > 100) {
			stepTurn(Parameters.Direction.LEFT);
			state = TURN_SOUTH_TO_BYBASS_EAST_TASK_3;
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_3 && !(isSameDirection(getHeading(), Parameters.EAST))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_3 && isSameDirection(getHeading(), Parameters.EAST)) {
			state = TURN_SOUTH_TO_BYBASS_EAST_TASK_4;
			oldX = (int) myX;
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_4 && myX - oldX <= 350) {
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_4 && myX - oldX > 350) {
			stepTurn(Parameters.Direction.LEFT);
			state = TURN_SOUTH_TO_BYBASS_EAST_TASK_5;
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_5 && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_5 && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = TURN_SOUTH_TO_BYBASS_EAST_TASK_6;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_6 && oldY - myY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_6 && oldY - myY > 100) {
			stepTurn(Parameters.Direction.LEFT);
			state = TURN_SOUTH_TO_BYBASS_EAST_TASK_7;
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_7 && !(isSameDirection(getHeading(), Parameters.WEST))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_EAST_TASK_7 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = MOVE_BACK_AND_FIRE_TASK;
			return;
		}

	}

	private void bypassWreck_N_W() {

		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_1 && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_1 && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = TURN_NORTH_TO_BYBASS_WEST_TASK_2;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_2 && oldY - myY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_2 && oldY - myY > 100) {
			stepTurn(Parameters.Direction.LEFT);
			state = TURN_NORTH_TO_BYBASS_WEST_TASK_3;
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_3 && !(isSameDirection(getHeading(), Parameters.WEST))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_3 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = TURN_NORTH_TO_BYBASS_WEST_TASK_4;
			oldX = (int) myX;
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_4 && oldX - myX <= 350) {
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_4 && oldX - myX > 350) {
			stepTurn(Parameters.Direction.LEFT);
			state = TURN_NORTH_TO_BYBASS_WEST_TASK_5;
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_5 && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_5 && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = TURN_NORTH_TO_BYBASS_WEST_TASK_6;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_6 && myY - oldY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_6 && myY - oldY > 100) {
			stepTurn(Parameters.Direction.RIGHT);
			state = TURN_NORTH_TO_BYBASS_WEST_TASK_7;
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_7 && !(isSameDirection(getHeading(), Parameters.WEST))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_NORTH_TO_BYBASS_WEST_TASK_7 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = MOVE_AND_FIRE_TASK;
			return;
		}
	}

	private void bypassWreck_S_W() {
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_1 && !(isSameDirection(getHeading(), Parameters.SOUTH))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_1 && isSameDirection(getHeading(), Parameters.SOUTH)) {
			state = TURN_SOUTH_TO_BYBASS_WEST_TASK_2;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_2 && myY - oldY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_2 && myY - oldY > 100) {
			stepTurn(Parameters.Direction.RIGHT);
			state = TURN_SOUTH_TO_BYBASS_WEST_TASK_3;
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_3 && !(isSameDirection(getHeading(), Parameters.WEST))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_3 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = TURN_SOUTH_TO_BYBASS_WEST_TASK_4;
			oldX = (int) myX;
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_4 && oldX - myX <= 350) {
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_4 && oldX - myX > 350) {
			stepTurn(Parameters.Direction.RIGHT);
			state = TURN_SOUTH_TO_BYBASS_WEST_TASK_5;
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_5 && !(isSameDirection(getHeading(), Parameters.NORTH))) {
			stepTurn(Parameters.Direction.RIGHT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_5 && isSameDirection(getHeading(), Parameters.NORTH)) {
			state = TURN_SOUTH_TO_BYBASS_WEST_TASK_6;
			oldY = (int) myY;
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_6 && oldY - myY <= 100) {
			myMove();
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_6 && oldY - myY > 100) {
			stepTurn(Parameters.Direction.LEFT);
			state = TURN_SOUTH_TO_BYBASS_WEST_TASK_7;
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_7 && !(isSameDirection(getHeading(), Parameters.WEST))) {
			stepTurn(Parameters.Direction.LEFT);
			return;
		}
		if (state == TURN_SOUTH_TO_BYBASS_WEST_TASK_7 && isSameDirection(getHeading(), Parameters.WEST)) {
			state = MOVE_AND_FIRE_TASK;
			return;
		}
	}

}
