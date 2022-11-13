package roscobp;

import il.ac.bgu.cs.bp.bpjs.execution.BProgramRunner;
import il.ac.bgu.cs.bp.bpjs.execution.listeners.PrintBProgramRunnerListener;
import il.ac.bgu.cs.bp.bpjs.model.BProgram;
import il.ac.bgu.cs.bp.bpjs.model.eventselection.PrioritizedBSyncEventSelectionStrategy;
import il.ac.bgu.cs.bp.bpjs.context.*;

public class Turtlebot3Main {

    public static void main(String[] args) {
        // alternate files, this program would only perform collision avoidance
        //final BProgram bprog = new ContextBProgram("collision-avoidance/cobp_sim.dal.js", "collision-avoidance/cobp_sim.bl.js");

        // initializes the COBPjs-Program with the specified files
        final BProgram bprog = new ContextBProgram("cobp_sim.dal.js", "cobp_sim.bl.js");
        // waits for external events that contain the data from the TurtleBot
        bprog.setWaitForExternalEvents(true);

        BProgramRunner rnr = new BProgramRunner(bprog);
        // employs an event selection strategy the allows to prioritize events based on a priority value
        // that is added to their sync statement
        bprog.setEventSelectionStrategy(new PrioritizedBSyncEventSelectionStrategy());
        // adds listener that prints information like the selected event or the creation of Live-Copies to the console
        rnr.addListener(new PrintBProgramRunnerListener());

        final RosBridge rosBridge = new RosBridge(bprog, rnr);
        // makes the rosBridge object accessible by the COBPjs code
        bprog.putInGlobalScope("ros", rosBridge);

        // starts the programm
        rnr.run();
    }
}
