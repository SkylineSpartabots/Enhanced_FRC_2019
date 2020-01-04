/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;
import frc.robot.subsystems.requests.RequestList;
import frc.robot.states.DriveSignal;

/**
 * Add your docs here.
 */
public class Superstructure extends Subsystem {
    private static Superstructure instance = null;

    public static Superstructure getInstance() {
        if (instance == null)
            instance = new Superstructure();
        return instance;
    }

    private Intake intake;
    private HatchMechanism hatchMech;
    private Elevator elevator;
    private Drivetrain drive;
    private Compressor compressor;

    private Superstructure() {
        intake = Intake.getInstance();
        hatchMech = HatchMechanism.getInstance();
        elevator = Elevator.getInstance();
        drive = Drivetrain.getInstance();

        compressor = new Compressor();
        compressor.setClosedLoopControl(true);

        queuedRequests = new ArrayList<>(0);

    }

    public enum ElevatorHeights {
        DOWN(0.0, 0.0), FIRST_LEVEL(0.0, 22.5), SECOND_LEVEL(33.5, 52.5), THIRD_LEVEL(63.5, 79), CARGO_SHIP(40, 40);
 
        public double hatchPosition;
        public double cargoPosition;

        private ElevatorHeights(double hatchPosition, double cargoPosition) {
            this.hatchPosition = hatchPosition;
            this.cargoPosition = cargoPosition;
        }

        public double getHeight() {
            if(hasHatchByDeduction()) {
                return hatchPosition;
            }
            return cargoPosition;
        }
    }

    private RequestList activeRequests;
    private ArrayList<RequestList> queuedRequests;
    private Request currentRequest;

    private boolean newRequests = false;
    private boolean activeRequestsCompleted = false;
    private boolean allRequestsCompleted = false;

    public boolean requestsCompleted() {
        return allRequestsCompleted;
    }

    private void setQueuedRequests(RequestList requests) {
        queuedRequests.clear();
        queuedRequests.add(requests);
    }

    private void setQueuedRequests(List<RequestList> requests) {
        queuedRequests.clear();
        queuedRequests = new ArrayList<>(requests.size());
        for (RequestList list : requests) {
            queuedRequests.add(list);
        }
    }

    private void setActiveRequests(RequestList requests) {
        activeRequests = requests;
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void request(Request r) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueuedRequests(new RequestList());
    }

    public void request(Request active, Request queue) {
        setActiveRequests(new RequestList(Arrays.asList(active), false));
        setQueuedRequests(new RequestList(Arrays.asList(queue), false));
    }

    public void request(RequestList requestList) {
        setActiveRequests(requestList);
        setQueuedRequests(new RequestList());
    }

    public void request(RequestList activeList, RequestList queuedList) {
        setActiveRequests(activeList);
        setQueuedRequests(queuedList);
    }

    public void addActiveRequest(Request request) {
        activeRequests.add(request);
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void addFormostActiveRequest(Request request) {
        activeRequests.addToForeFront(request);
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void queue(Request request) {
        queuedRequests.add(new RequestList(Arrays.asList(request), false));
    }

    public void queue(RequestList list) {
        queuedRequests.add(list);
    }

    public void replaceQueue(Request request) {
        setQueuedRequests(new RequestList(Arrays.asList(request), false));
    }

    public void replaceQueue(RequestList list) {
        setQueuedRequests(list);
    }

    public void replaceQueue(List<RequestList> lists) {
        setQueuedRequests(lists);
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            stop();
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                if (!activeRequestsCompleted) {
                    if (newRequests) {
                        if (activeRequests.isParallel()) {
                            boolean allActivated = true;
                            for (Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator
                                    .hasNext();) {
                                Request request = iterator.next();
                                boolean allowed = request.allowed();
                                allActivated &= allowed;
                                if (allowed)
                                    request.act();
                            }
                            newRequests = !allActivated;
                        } else {
                            if (activeRequests.isEmpty()) {
                                activeRequestsCompleted = true;
                                return;
                            }
                            currentRequest = activeRequests.remove();
                            currentRequest.act();
                            newRequests = false;
                        }
                    }
                    if (activeRequests.isParallel()) {
                        boolean done = true;
                        for (Request request : activeRequests.getRequests()) {
                            done &= request.isFinished();
                        }
                        activeRequestsCompleted = done;
                    } else if (currentRequest.isFinished()) {
                        if (activeRequests.isEmpty()) {
                            activeRequestsCompleted = true;
                        } else if (activeRequests.getRequests().get(0).allowed()) {
                            newRequests = true;
                            activeRequestsCompleted = false;
                        }
                    }
                } else {
                    if (!queuedRequests.isEmpty()) {
                        setActiveRequests(queuedRequests.remove(0));
                    } else {
                        allRequestsCompleted = true;
                    }
                }

            }
        }

        @Override
        public void onStop(double timestamp) {

        }

    };

    public RequestList disabledRequest() {
        return new RequestList(Arrays.asList(drive.openLoopRequest(DriveSignal.BRAKE), elevator.openLoopRequest(0.0), intake.stateRequest(Intake.State.OFF),
                hatchMech.stateRequest(HatchMechanism.State.STOWED)), true);
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {
        setActiveRequests(disabledRequest());
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    public void iaTesting() {
        RequestList state = new RequestList(Arrays.asList(elevator.timedPowerSet(0.55, 0.5)), true);
        RequestList queue = new RequestList(Arrays.asList(elevator.logVelocity("/home/lvuser/IATesting/logfile" + new Date().toString() + ".txt"), elevator.timedPowerSet(0, 0.1)), true);
        request(state);
        replaceQueue(queue);
    }

    public void hatchRetrievingState() {
        RobotState.visionTarget.setDesiredTargetArea(9);
        RequestList state = new RequestList(Arrays.asList(drive.alignToTargetRequest(),
                elevator.heightRequest(ElevatorHeights.DOWN.hatchPosition),
                hatchMech.stateRequest(HatchMechanism.State.RECIEVING), intake.stateRequest(Intake.State.CARGO_PHOBIC)),
                true);
        RequestList queue = new RequestList(Arrays.asList(driveUntilHatchRequest(), waitRequest(0.3),
                drive.openLoopRequest(new DriveSignal(-0.15, -0.15)), waitRequest(0.5),
                drive.openLoopRequest(DriveSignal.BRAKE), hatchMech.stateRequest(HatchMechanism.State.STOWED),
                intake.stateRequest(Intake.State.OFF)), false);
        request(state);
        replaceQueue(queue);
    }

    public void deployingState(ElevatorHeights height) {
        RobotState.visionTarget.setDesiredTargetArea(8.7);
        if (RobotState.visionTarget.isTargetVisible()) {
            if (hasHatchByDeduction()) {
                //hatch deploy
                RequestList state = new RequestList(Arrays.asList(drive.alignToTargetRequest(), elevator.heightRequest(ElevatorHeights.DOWN.hatchPosition),
                        hatchMech.stateRequest(HatchMechanism.State.STOWED), intake.stateRequest(Intake.State.OFF)),
                        true);
                RequestList queue = new RequestList(Arrays.asList(elevator.heightRequest(height.hatchPosition),
                        drive.timeDriveRequest(new DriveSignal(0.25, 0.25), 0.9),
                        drive.openLoopRequest(DriveSignal.NEUTRAL), hatchMech.stateRequest(HatchMechanism.State.SCORING),
                        waitRequest(0.25), drive.timeDriveRequest(new DriveSignal(-0.2, -0.2), 0.75),
                        drive.openLoopRequest(DriveSignal.BRAKE), hatchMech.stateRequest(HatchMechanism.State.STOWED),
                        waitRequest(0.2), elevator.heightRequest(ElevatorHeights.DOWN.hatchPosition)), false);
                request(state);
                replaceQueue(queue);
            } else {
                //cargo deploy
                RequestList state = new RequestList(Arrays.asList(drive.alignToTargetRequest(),
                        hatchMech.stateRequest(HatchMechanism.State.STOWED), intake.stateRequest(Intake.State.OFF)),
                        true);
                RequestList queue = new RequestList(Arrays.asList(
                        elevator.heightRequest(height.cargoPosition),
                        drive.timeDriveRequest(new DriveSignal(0.25, 0.25), 0.9),
                        drive.openLoopRequest(DriveSignal.BRAKE), intake.stateRequest(Intake.State.OUTAKE_ELEVATOR_UP),
                        waitRequest(0.3), intake.stateRequest(Intake.State.OFF),
                        elevator.heightRequest(ElevatorHeights.DOWN.hatchPosition)), false);
                request(state);
                replaceQueue(queue);
            }
        }

    }

    public void alignToTarget() {
        if(RobotState.visionTarget.isTargetVisible()) {
            request(drive.alignToTargetRequest());
        }
    }

    public void clearRequests() {
        activeRequests = new RequestList();
        queuedRequests.clear();
    }


    private static boolean hasHatchByDeduction() {
        if (Intake.getInstance().hasCargo() && !HatchMechanism.getInstance().hasHatch()) {
            return false;
        }
        return true;
    }

    public Request waitRequest(double seconds) {
        return new Request() {
            double startTime = 0;
            double waitTime = 1;

            @Override
            public void act() {
                startTime = Timer.getFPGATimestamp();
                waitTime = seconds;
            }

            @Override
            public boolean isFinished() {
                return (Timer.getFPGATimestamp() - startTime) >= waitTime;
            }
        };
    }

    public Request driveUntilHatchRequest() {
        return new Request() {
            double startTime = 0;
            double timeOut = 2;

            @Override
            public void act() {
                startTime = Timer.getFPGATimestamp();
                drive.setOpenLoop(new DriveSignal(0.2, 0.2));
                hatchMech.conformToState(HatchMechanism.State.RECIEVING);
            }

            @Override
            public boolean isFinished() {
                return hatchMech.hasHatch() || (Timer.getFPGATimestamp() - startTime) >= timeOut;
            }
        };
    }

}
