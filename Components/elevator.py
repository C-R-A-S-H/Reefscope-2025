import rev
import phoenix6
import wpilib

sigMIN = -7
sigMAX = 123


class Elevator():
    def __init__(self):
        self.kracken1 = phoenix6.hardware.TalonFX(19, "rio")
        self.kracken2 = phoenix6.hardware.TalonFX(20, "rio")
        self.vortex = rev.SparkFlex(22, rev.SparkFlex.MotorType.kBrushless)

        self.startPOS = self.kracken1.get_rotor_position().value_as_double + 20
        self.maxPOS = self.startPOS + 50

        self.snaps = {
            'A': 48,
            'B': 40,
            'Y': 70,
            'X': 115
        }  # Assign levels to buttons
        self.snapLocat = None
        self.isSnapping = False
        self.snapThreshold = 3  # Increased threshold to reduce jitter
        self.minPower = 0.15  # Minimum power to prevent weak movements
        self.maxPower = 0.35  # Lower max power to reduce overshooting

    def getCurrentPosition(self):
        return self.kracken1.get_rotor_position().value_as_double

    def EleExtend(self, power):
        power = max(-self.maxPower, min(self.maxPower, power))
        current_pos = self.getCurrentPosition()

        # Safety checks for range limits
        if current_pos > self.startPOS and power < 0:
            self.kracken1.set(power)
            self.kracken2.set(power)
        elif current_pos < self.maxPOS and power > 0:
            self.kracken1.set(power)
            self.kracken2.set(power)
        else:
            self.kracken1.set(0)
            self.kracken2.set(0)

    def CoralEater(self, power):
        self.vortex.set(power)

    def Stop(self):
        self.kracken1.disable()
        self.kracken2.disable()
        self.vortex.disable()

    def Disable(self):
        self.kracken1.disable()
        self.kracken2.disable()
        self.kracken1.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        self.kracken2.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        self.vortex.disable()

    def Enable(self):
        self.startPOS = self.getCurrentPosition() + 20
        self.maxPOS = self.startPOS + 106
        self.kracken1.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.kracken2.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

    def snapToLevel(self, button):
        if button in self.snaps:
            self.snapLocat = self.snaps[button]
            self.isSnapping = True

    def updateSnap(self):
        if self.isSnapping:
            current_pos = round(self.getCurrentPosition())
            error = self.snapLocat - current_pos

            if abs(error) <= self.snapThreshold:
                self.EleExtend(0)
                self.isSnapping = False
                return

            direction = 1 if error > 0 else -1
            power = max(self.minPower, min(self.maxPower, abs(error) / 15)) * direction  # Smooth deceleration

            self.EleExtend(power)


    def getLimit1(self):
        self.limit1.get()

    def getLimit2(self):
        self.limit2.get()

    def getLimit3(self):
        self.limit3.get()

    def getLimit4(self):
        self.limit4.get()


    def set_intake_power(self, power):
        self.vortex.set(power/3)
