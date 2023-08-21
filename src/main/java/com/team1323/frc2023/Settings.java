/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023;

import com.team1323.frc2023.field.FieldOffsets;
import com.team1323.frc2023.field.PracticeFieldOffsets;

/**
 * 
 */
public class Settings {
    public static final boolean kIsUsingCompBot = true;
    public static final boolean kIsUsingShoulderCANCoder = false;
    public static final boolean kIsUsingPS5Controller = true;
    public static final FieldOffsets kFieldOffsets = new PracticeFieldOffsets();

    public static final boolean kSimulate = false;
	public static final boolean kResetTalons = false;
}
