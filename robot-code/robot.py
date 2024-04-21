#!/usr/bin/env python3

"""
Base de code Tank Drive + demo component
"""

import wpilib
from components import demo_component, three_wheel_drive
from magicbot import MagicRobot
import constants
from navx import AHRS
import rev


class MyRobot(MagicRobot):
    """
    Après avoir créer notre composante 'demo_component' de bas niveau dans './components/demo_component.py',
    utiliser leur nom suivi d'un trait souligné (_) pour injecter des objets au composantes.

    Utilisez le signe = dans la fonction 'createObjects' pour vous assurer que les données sont biens transmises à leur composantes.

    Pour plus d'information (en anglais): https://robotpy.readthedocs.io/en/stable/frameworks/magicbot.html
    """

    # Ici on définie l'objet qui nous permet d'interragir avec le component
    demo_component: demo_component.DemoComponent
    drivetrain: three_wheel_drive.ThreeWheelDrive

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "drivetrain_" vont être injectés.
        """
        # Injection des moteurs dans le drivetrain
        self.drivetrain_left_motor = rev.CANSparkMax(constants.PWM_TANK_DRIVE_LEFT, rev.CANSparkMax.MotorType.kBrushless)
        self.drivetrain_right_motor = rev.CANSparkMax(constants.PWM_TANK_DRIVE_RIGHT, rev.CANSparkMax.MotorType.kBrushless)
        self.drivetrain_center_motor = rev.CANSparkMax(constants.PWM_TANK_DRIVE_CENTER, rev.CANSparkMax.MotorType.kBrushless)
        self.navx = AHRS(wpilib.SerialPort.Port.kMXP)

        # General
        self.gamepad1 = wpilib.XboxController(0)

    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        pass

    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        pass

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode téléopéré."""
        pass

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""
        self.drivetrain.controller_move(self.gamepad1.getLeftY(), self.gamepad1.getLeftX(), self.gamepad1.getRightX())

        if self.gamepad1.getAButton():
            self.demo_component.set_speed(1)
