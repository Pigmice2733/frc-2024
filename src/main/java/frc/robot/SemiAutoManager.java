package frc.robot;

import java.util.PriorityQueue;
import java.util.Queue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SemiAutoManager {
    private Command currentTask;
    private Queue<Command> taskQueue = new PriorityQueue<Command>();

    /** Adds a new task to the end of the queue */
    public void queueTask(Command command) {
        taskQueue.add(command);

        if (currentTask == null)
            startNextTask();
    }

    /** Skips to the next task in the queue, and ends the current task */
    public void startNextTask() {
        // End the current task
        if (currentTask != null)
            currentTask.cancel();

        // Grab the next task from the queue
        var nextTask = taskQueue.poll();

        // Start the next task
        if (nextTask != null)
            nextTask.andThen(onCurrentTaskFinished()).schedule();

        currentTask = nextTask;
    }

    /** A task to be run after a task is complete, starting the next */
    private Command onCurrentTaskFinished() {
        return Commands.runOnce(() -> startNextTask());
    }

    /** Returns the task that is currently running */
    public Command getCurrentTask() {
        return currentTask;
    }

    /** Returns the task at the front of the queue, next to be scheduled */
    public Command peekAtNextTask() {
        return taskQueue.peek();
    }

    /** Pauses the current task */
    public void pause() {
        if (currentTask != null)
            currentTask.cancel();
    }

    /** Resumes the current task */
    public void resume() {
        if (currentTask != null && !currentTask.isScheduled())
            currentTask.schedule();
    }
}
