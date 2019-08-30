/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Limelight;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;
import frc.robot.subsystems.requests.RequestList;

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
        DOWN(0, 0), FIRST_LEVEL(0, 20.5), SECOND_LEVEL(28, 48.5), THIRD_LEVEL(56, 76.5), CARGO_SHIP(0, 40);

        public double hatchPosition;
        public double cargoPosition;

        private ElevatorHeights(double hatchPosition, double cargoPosition) {
            this.hatchPosition = hatchPosition;
            this.cargoPosition = cargoPosition;
        }

        public double getHeight() {
            if (Intake.getInstance().hasCargo() && !HatchMechanism.getInstance().hasHatch()) {
                return cargoPosition;
            }
            return hatchPosition;

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

        @Override
        public void onStop(double timestamp) {

        }

    };

    public RequestList disabledRequest() {
        return new RequestList(Arrays.asList(elevator.openLoopRequest(0.0), intake.stateRequest(Intake.State.OFF),
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

    public void hatchRetrievingState() {
        RequestList state = new RequestList(Arrays.asList(drive.alignToTargetRequest(),
                elevator.heightRequest(ElevatorHeights.FIRST_LEVEL.hatchPosition),
                hatchMech.stateRequest(HatchMechanism.State.RECIEVING), intake.stateRequest(Intake.State.CARGO_PHOBIC)),
                true);
        request(state);
    }

    public void requestTest() {
        RequestList state = new RequestList(
                    Arrays.asList(hatchMech.stateRequest(HatchMechanism.State.SCORING), intake.stateRequest(Intake.State.IDLE_WITH_KEBABS)),
                    true);
        RequestList queue = new RequestList(Arrays.asList(waitRequest(2), hatchMech.stateRequest(HatchMechanism.State.STOWED),
                    waitRequest(2), intake.stateRequest(Intake.State.IDLE_WITH_KEBABS)), false);
        request(state);
        replaceQueue(queue);
    }

    public void deployingState(ElevatorHeights height) {
        if (HatchMechanism.getInstance().hasHatch()) {
            RequestList state = new RequestList(
                    Arrays.asList(drive.alignToTargetRequest(), elevator.heightRequest(height.hatchPosition),
                            hatchMech.stateRequest(HatchMechanism.State.STOWED), intake.stateRequest(Intake.State.OFF)),
                    true);
            RequestList queue = new RequestList(Arrays.asList(hatchMech.stateRequest(HatchMechanism.State.SCORING),
                    waitRequest(0.3), hatchMech.stateRequest(HatchMechanism.State.FINGERS_EXTENDED_RETRACTED), waitRequest(0.15),
                    hatchMech.stateRequest(HatchMechanism.State.STOWED), waitRequest(0.1), elevator.heightRequest(ElevatorHeights.DOWN.hatchPosition)), false);
            request(state);
            replaceQueue(queue);
        } else if (Intake.getInstance().hasCargo()) {
            RequestList state = new RequestList(
                    Arrays.asList(drive.alignToTargetRequest(), elevator.heightRequest(height.cargoPosition),
                            hatchMech.stateRequest(HatchMechanism.State.STOWED), intake.stateRequest(Intake.State.OFF)),
                    true);
            RequestList queue = new RequestList(Arrays.asList(intake.stateRequest(Intake.State.OUTAKE_ELEVATOR_UP),
                    waitRequest(0.3), intake.stateRequest(Intake.State.OFF),
                    elevator.heightRequest(ElevatorHeights.DOWN.hatchPosition)), false);
            request(state);
            replaceQueue(queue);
        }

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
}
