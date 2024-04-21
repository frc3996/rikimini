
import wpilib
from wpimath import controller, geometry
from magicbot import tunable, will_reset_to, feedback
import constants
from dataclasses import dataclass
import math
import rev
from navx import AHRS


@dataclass
class Command:
    forward: float = 0
    strafe: float = 0
    rotate: float = 0


class ThreeWheelDrive:
    left_motor: rev.CANSparkMax
    right_motor: rev.CANSparkMax
    center_motor: rev.CANSparkMax
    navx: AHRS
    max_speed = tunable(0.25)
    max_rot_speed = tunable(0.05)
    __controller_command = will_reset_to(Command())
    __auto_command = will_reset_to(None)
    angle_kp = tunable(0.2)
    angle_ki = tunable(0)
    angle_kd = tunable(0)

    def setup(self):
        """
        Appelé après l'injection
        """
        self.left_motor.restoreFactoryDefaults()
        self.right_motor.restoreFactoryDefaults()
        self.center_motor.restoreFactoryDefaults()

        self.left_motor.setInverted(True)
        self.right_motor.setInverted(True)
        self.center_motor.setInverted(True)
        self.left_motor.setOpenLoopRampRate(1)
        self.right_motor.setOpenLoopRampRate(1)
        self.center_motor.setOpenLoopRampRate(1)

        self.angle_pid = controller.PIDController(self.angle_kp, self.angle_ki, self.angle_kd)
        self.angle_pid.enableContinuousInput(-math.pi, math.pi)
        self.target_angle = self.navx.getRotation2d()

    def on_enable(self):
        self.angle_pid.setP(self.angle_kp)
        self.angle_pid.setI(self.angle_ki)
        self.angle_pid.setD(self.angle_kd)
        self.target_angle = self.navx.getRotation2d()


    def controller_move(self, forward, strafe, rotate):
        """
        Déplace le robot en teleop
        Forwared positif vers l'avant
        Strafe positif vers la gauche
        Rotate position anti-horaire
        """
        # On inverse les joystick (-1 est vers le haut)
        forward_speed = -forward
        strafe_speed = -strafe
        rotation_speed = -rotate

        # On retire les petites valeurs pour que le robot ne bouge pas si on ne touche pas au joystick
        if abs(forward_speed) < constants.CONTROLLER_LOWER_INPUT_THRESHOLD:
            forward_speed = 0

        if abs(strafe_speed) < constants.CONTROLLER_LOWER_INPUT_THRESHOLD:
            strafe_speed = 0

        if abs(rotation_speed) < constants.CONTROLLER_LOWER_INPUT_THRESHOLD:
            rotation_speed = 0

        # On multiple par le facteur de vitesse
        forward_speed = forward_speed * self.max_speed
        strafe_speed = strafe_speed * self.max_speed
        rotation_speed = rotation_speed * self.max_rot_speed

        self.__controller_command = Command(forward_speed, strafe_speed, rotation_speed)

    def auto_move(self, forward, strafe, rotate):
        """
        Déplace le robot en mode autonome
        Forwared positif vers l'avant
        Strafe positif vers la gauche
        Rotate position anti-horaire
        """
        self.__auto_command = Command(forward, strafe, rotate)

    @feedback
    def current_target_angle(self):
        return self.target_angle.degrees()

    @feedback
    def current_angle(self):
        return self.navx.getRotation2d().degrees()

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """

        command = self.__controller_command
        if self.__auto_command != None:
            command = self.__auto_command

        # W1 = -1/2 X - sqrt(3)/2 Y + R
        # W2 = -1/2 X + sqrt(3)/2 Y + R
        # W3 = X + R

        self.target_angle += geometry.Rotation2d(command.rotate)

        rot_error = self.angle_pid.calculate(self.navx.getRotation2d().radians(), self.target_angle.radians())

        W1 = -0.5 * command.strafe - math.sqrt(3)/2 * command.forward + rot_error
        W2 = -0.5 * command.strafe + math.sqrt(3)/2 * command.forward + rot_error
        W3 = command.strafe + rot_error

        self.left_motor.set(W1)
        self.right_motor.set(W2)
        self.center_motor.set(W3)
