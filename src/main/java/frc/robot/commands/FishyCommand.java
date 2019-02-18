package frc.robot.commands;

import frc.robot.util.logging.CommandLogger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This Command subclass gives us everything we want out of a command... and more.
 */
public abstract class FishyCommand extends Command {
    protected CommandLogger logger;

    protected FishyCommand(Subsystem... subsystems) {
        super();
        for(Subsystem s : subsystems) {
            requires(s);
        }
        logger = new CommandLogger(this.getName());
        String[] dataElements = getLogFields();
        for (String de : dataElements) {
            logger.addDataElement(de);
        }
    }

    protected abstract String[] getLogFields();

    protected void log(String field, String value) {
        logger.set(field, value);
    }

    protected void log(String field, double value) {
        logger.set(field, value);
    }

    @Override
    protected void execute() {
        super.execute();
    }

    @Override
    protected void end() {
        super.end();
        logger.drain();
        logger.flush();
    }
}