/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.raisin;

/**
 * Generator represents a motor output generator, based on some raisin path or
 * equivalent user input.
 *
 * @author Dowland Aiello
 **/
public interface Generator {
    /**
     * Registers the given modifier method on the generator.
     **/
    public void registerModifier(Modifier m);

    /**
     * Gets the percentage output for each of the registered inputs.
     *
     * @return The percentage speed returned by each registered input
     **/
    public double[] percentageOutputs();
}
