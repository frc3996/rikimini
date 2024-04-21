from .base_auto import BaseAuto
from magicbot.state_machine import timed_state
from components import demo_component, three_wheel_drive


class ForwardAndReverse(BaseAuto):
    MODE_NAME = "ForwardAndReverse"
    DEFAULT = True

    # Injection
    drivetrain: three_wheel_drive.ThreeWheelDrive
    demo_component: demo_component.DemoComponent

    @timed_state(duration=1, next_state="do_demo", first=True)
    def forward(self):
        # Avance 1 seconde et arrête
        self.drivetrain.auto_move(1, 0, 0)

    @timed_state(duration=1, next_state="reverse")
    def do_demo(self):
        # Fait quelque chose 1 seconde et arrête
        self.demo_component.set_speed(1)

    @timed_state(duration=1, next_state="finish")
    def reverse(self):
        # Recule 1 seconde et arrête
        self.drivetrain.auto_move(-1, 0, 0)
