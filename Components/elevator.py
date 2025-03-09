import rev
import phoenix6
import wpilib

class Elevator():
    def __init__(self):
        self.kracken1 = phoenix6.hardware.TalonFX(19, "rio")
        self.kracken2 = phoenix6.hardware.TalonFX(20, "rio")
        self.minineo = rev.SparkMax(22, rev.SparkMax.MotorType.kBrushless)

        self.limit1 = wpilib.DigitalInput(0)
        self.limit2 = wpilib.DigitalInput(1)
        self.limit3 = wpilib.DigitalInput(2)
        self.limit4 = wpilib.DigitalInput(3)

    def EleExtend(self, power):
        self.kracken1.set(power)
        self.kracken2.set(-power)

    def CoralEater(self, power):
        self.minineo.set(power)

    def getLimit1(self):
        self.limit1.get()

    def getLimit2(self):
        self.limit2.get()

    def getLimit3(self):
        self.limit3.get()

    def getLimit4(self):
        self.limit4.get()

    def Disable(self):
        self.kracken.disable()

    def Update(self):
        self.kracken.get_position()

    def Stop(self):
        self.kracken.disable()