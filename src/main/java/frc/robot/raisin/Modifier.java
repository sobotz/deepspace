/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.raisin;

/**
 * Modifier represents a functional modification operator on a given input.
 *
 * @author Dowland Aiello
 **/
public interface Modifier {
    /**
     * Applies the modifier to the given decimal input.
     *
     * @param d The inputted decimal value
     * @return The modified inputted decimal
     **/
    double apply(double d);
}
