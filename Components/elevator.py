
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
        self.maxPOS = self.startPOS + 50#116
        self.sillyDouble = self.kracken1.get_rotor_position().value_as_double

        self.limit1 = wpilib.DigitalInput(0)
        self.limit2 = wpilib.DigitalInput(1)
        self.limit3 = wpilib.DigitalInput(2)
        self.limit4 = wpilib.DigitalInput(3)

    def EleExtend(self, power):

        # Assuming ele is at bottom

        # self.kracken1.set(power)
        # self.kracken2.set(-power)

        if self.kracken1.get_rotor_position().value_as_double > self.startPOS and power < 0:
            self.kracken1.set(power)
            self.kracken2.set(power)
        elif self.kracken1.get_rotor_position().value_as_double < self.maxPOS and power > 0:
            self.kracken1.set(power)
            self.kracken2.set(power)
        else:
            self.kracken1.disable()
            self.kracken2.disable()

        # request1 = phoenix6.controls.PositionDutyCycle(10, 1, False)
        # request2 = phoenix6.controls.PositionDutyCycle(10, 1, False)
        #y
        # self.kracken1.set_control(request1)
        # self.kracken2.set_control(request2)
    def CoralEater(self, power):
        self.vortex.set(power)

    def getLimit1(self):
        self.limit1.get()

    def getLimit2(self):
        self.limit2.get()

    def getLimit3(self):
        self.limit3.get()

    def getLimit4(self):
        self.limit4.get()

    def Disable(self):
        self.kracken1.disable()
        self.kracken2.disable()
        self.kracken1.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        self.kracken2.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)

    def Update(self):
        self.kracken.get_position()

    def Stop(self):
        self.kracken1.disable()
        self.kracken2.disable()

    def Enable(self):
        self.startPOS = self.kracken1.get_rotor_position().value_as_double + 20
        self.maxPOS = self.startPOS + 50  # 116
        self.kracken1.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.kracken2.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)