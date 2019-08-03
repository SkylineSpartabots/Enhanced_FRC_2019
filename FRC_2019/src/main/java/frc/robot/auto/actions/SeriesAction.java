/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Add your docs here.
 */
public class SeriesAction implements Action {

    private Action currentAction;
    private final ArrayList<Action> remainingActions;

    public SeriesAction(List<Action> actions) {
        remainingActions = new ArrayList<>(actions);
        currentAction = null;
    }

    @Override
    public boolean isFinished() {
        return remainingActions.isEmpty() && currentAction == null;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        if(currentAction == null) {
            if(remainingActions.isEmpty()) {
                return;
            }

            currentAction = remainingActions.remove(0);
            currentAction.start();
        }

        if(currentAction.isFinished()) {
            currentAction.isFinished();
            currentAction = null;
        }
    }

    @Override
    public void done() {

    }
}
