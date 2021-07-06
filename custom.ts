
const enum PinooBitMotor {
  //% block="A"
  A = 0,
  //% block="B"
  B = 1,
  //% block="A + B"
  All = 2,
}

const enum PinooBitMotorRotation {
  //% block="forward"
  Forward = 1,
  //% block="backward"
  Backward = -1,
}

//% color=#0fbc11 icon="\u272a" block="PinooBit"

  const motorRotations = [
    PinooBitMotorRotation.Forward,
    PinooBitMotorRotation.Forward,
  ];
//% color="#18D4D6"
//% category="PinooBit" icon="♾"
namespace pinooBit {
let distance = 0
	
	// PinooBit motor driver blocks

  /**
   * Sets the speed of a motor.
   * @param motor motor, eg: PinooBitMotor.A
   * @param speed percentage in the range of -100 to 100, eg: 80
   */
  //% subcategory=Motors
  //% blockId="PinooBit_motor_run" block="run motor %motor | at speed %speed \\%"
  //% speed.min=-100
  //% speed.max=100
  //% weight=90
  export function runMotor(motor: PinooBitMotor, speed: number): void {
    if (speed === 0) {
      stopMotor(motor);
      return;
    }

    const absSpeedPercentage = Math.min(Math.abs(speed), 100);
    const analogSpeed = pins.map(absSpeedPercentage, 0, 100, 0, 1023);

    if (motor === PinooBitMotor.A || motor === PinooBitMotor.All) {
      const isClockwise = speed * motorRotations[PinooBitMotor.A] > 0;
      pins.digitalWritePin(DigitalPin.P11, isClockwise ? 1 : 0);
      pins.digitalWritePin(DigitalPin.P12, isClockwise ? 0 : 1);
      if (speed === 100) {
        // Avoid PWM whenever possible as only 3 concurrent PWM outputs are available on the microbit
        pins.digitalWritePin(DigitalPin.P13, 1);
      } else {
        pins.analogWritePin(AnalogPin.P13, analogSpeed);
      }
    }

    if (motor === PinooBitMotor.B || motor === PinooBitMotor.All) {
      const isClockwise = speed * motorRotations[PinooBitMotor.B] > 0;
      pins.digitalWritePin(DigitalPin.P15, isClockwise ? 1 : 0);
      pins.digitalWritePin(DigitalPin.P16, isClockwise ? 0 : 1);
      if (speed === 100) {
        // Avoid PWM whenever possible as only 3 concurrent PWM outputs are available on the microbit
        pins.digitalWritePin(DigitalPin.P14, 1);
      } else {
        pins.analogWritePin(AnalogPin.P14, analogSpeed);
      }
    }
  }

  /**
   * Stops a motor.
   * @param motor motor, eg: PinooBitMotor.A
   */
  //% subcategory=Motors
  //% blockId="PinooBit_motor_stop" block="stop motor %motor"
  //% weight=89
  export function stopMotor(motor: PinooBitMotor): void {
    if (motor === PinooBitMotor.A || motor === PinooBitMotor.All) {
      pins.digitalWritePin(DigitalPin.P11, 0);
      pins.digitalWritePin(DigitalPin.P12, 0);
      pins.digitalWritePin(DigitalPin.P13, 0);
    }

    if (motor === PinooBitMotor.B || motor === PinooBitMotor.All) {
      pins.digitalWritePin(DigitalPin.P15, 0);
      pins.digitalWritePin(DigitalPin.P16, 0);
      pins.digitalWritePin(DigitalPin.P14, 0);
    }
  }

  /**
   * Sets the rotation direction of a motor. Use this function at start time to configure your motors without the need to rewire.
   * @param motor motor, eg: PinooBitMotor.A
   * @param rotation rotation of the motor, eg: PinooBitMotorRotation.Forward
   */
  //% subcategory=Motors
  //% blockId=PinooBit_motor_set_rotation block="set motor %motor rotation | to %rotation"
  //% weight=88
  export function setMotorRotation(
    motor: PinooBitMotor,
    rotation: PinooBitMotorRotation
  ) {
    if (motor === PinooBitMotor.A || motor === PinooBitMotor.All) {
      motorRotations[PinooBitMotor.A] = rotation;
    }

    if (motor === PinooBitMotor.B || motor === PinooBitMotor.All) {
      motorRotations[PinooBitMotor.B] = rotation;
    }
  }
  
//% subcategory=LCD
    let i2cAddr: number // 0x3F: PCF8574A, 0x27: PCF8574
    let BK: number      // backlight control
    let RS: number      // command/data

    // set LCD reg
	//% subcategory=LCD
    function setreg(d: number) {
        pins.i2cWriteNumber(i2cAddr, d, NumberFormat.Int8LE)
        basic.pause(1)
    }

    // send data to I2C bus
	//% subcategory=LCD
    function set(d: number) {
        d = d & 0xF0
        d = d + BK + RS
        setreg(d)
        setreg(d + 4)
        setreg(d)
    }

    // send command
	//% subcategory=LCD
    function cmd(d: number) {
        RS = 0
        set(d)
        set(d << 4)
    }

    // send data
	//% subcategory=LCD
    function dat(d: number) {
        RS = 1
        set(d)
        set(d << 4)
    }

    // auto get LCD address
	//% subcategory=LCD
    function AutoAddr() {
        let k = true
        let addr = 0x20
        let d1 = 0, d2 = 0
        while (k && (addr < 0x28)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr

        addr = 0x38
        while (k && (addr < 0x40)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr
        else return 0

    }

    /**
     * initial LCD, set I2C address. Address is 39/63 for PCF8574/PCF8574A
     * @param Addr is i2c address for LCD, eg: 0, 39, 63. 0 is auto find address
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_SET_ADDRESS" block="LCD initialize with Address %addr"
    //% weight=100 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function LcdInit(Addr: number) {
        if (Addr == 0) i2cAddr = AutoAddr()
        else i2cAddr = Addr
        BK = 8
        RS = 0
        cmd(0x33)       // set 4bit mode
        basic.pause(5)
        set(0x30)
        basic.pause(5)
        set(0x20)
        basic.pause(5)
        cmd(0x28)       // set mode
        cmd(0x0C)
        cmd(0x06)
        cmd(0x01)       // clear
    }

    /**
     * show a number in LCD at given position
     * @param n is number will be show, eg: 10, 100, 200
     * @param x is LCD column position, eg: 0
     * @param y is LCD row position, eg: 0
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_SHOW_NUMBER" block="show number %n|at x %x|y %y"
    //% weight=90 blockGap=8
    //% x.min=0 x.max=15
    //% y.min=0 y.max=1
    //% parts=LCD1602_I2C trackArgs=0
    export function ShowNumber(n: number, x: number, y: number): void {
        let s = n.toString()
        ShowString(s, x, y)
    }

    /**
     * show a string in LCD at given position
     * @param s is string will be show, eg: "Hello"
     * @param x is LCD column position, [0 - 15], eg: 0
     * @param y is LCD row position, [0 - 1], eg: 0
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_SHOW_STRING" block="show string %s|at x %x|y %y"
    //% weight=90 blockGap=8
    //% x.min=0 x.max=15
    //% y.min=0 y.max=1
    //% parts=LCD1602_I2C trackArgs=0
    export function ShowString(s: string, x: number, y: number): void {
        let a: number

        if (y > 0)
            a = 0xC0
        else
            a = 0x80
        a += x
        cmd(a)

        for (let i = 0; i < s.length; i++) {
            dat(s.charCodeAt(i))
        }
    }

    /**
     * turn on LCD
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_ON" block="turn on LCD"
    //% weight=81 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function on(): void {
        cmd(0x0C)
    }

    /**
     * turn off LCD
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_OFF" block="turn off LCD"
    //% weight=80 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function off(): void {
        cmd(0x08)
    }

    /**
     * clear all display content
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_CLEAR" block="clear LCD"
    //% weight=85 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function clear(): void {
        cmd(0x01)
    }

    /**
     * turn on LCD backlight
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_BACKLIGHT_ON" block="turn on backlight"
    //% weight=71 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function BacklightOn(): void {
        BK = 8
        cmd(0)
    }

    /**
     * turn off LCD backlight
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_BACKLIGHT_OFF" block="turn off backlight"
    //% weight=70 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function BacklightOff(): void {
        BK = 0
        cmd(0)
    }

    /**
     * shift left
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_SHL" block="Shift Left"
    //% weight=61 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function shl(): void {
        cmd(0x18)
    }

    /**
     * shift right
     */
	//% subcategory=LCD
    //% blockId="I2C_LCD1620_SHR" block="Shift Right"
    //% weight=60 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    export function shr(): void {
        cmd(0x1C)
    }

    //% subcategory=MainSensor
//% block="read ultrasonic distance sensor entry pin $entry out pin $out"
    export function distances(entry: DigitalPin, out: DigitalPin) {
    pins.digitalWritePin(entry, 0)
    control.waitMicros(2)
    pins.digitalWritePin(entry, 1)
    control.waitMicros(10)
    pins.digitalWritePin(entry, 0)
    distance = pins.pulseIn(entry, PulseValue.High) / 58
    basic.showNumber(distance)
    basic.pause(100)
    }
    //% subcategory=MainSensor
    //% color="#18D4D6"
    //% block="set led module pin %entryled value %ledvalue"
    //% ledvalue.min=0 ledvalue.max=1
    export function led(entryvalue: DigitalPin, ledvalue: number) {
            pins.digitalWritePin(entryvalue, ledvalue)
    }
    //% subcategory=MainSensor
    //% block="set buzzer module pin %entrybuzzer value %buzzervalue"
    //% buzzervalue.min=0 buzzervalue.max=1
    export function buzzer(entrybuzzer: DigitalPin, buzzervalue: number) {
            pins.digitalWritePin(entrybuzzer, buzzervalue)
    }
    //% subcategory=MainSensor
    //% block="set servo motor module pin %entryservo value %servovalue"
    //% servoDegeri.shadow="protractorPicker"
    export function servo(girisservo: DigitalPin, servoDegeri: number) {
            pins.digitalWritePin(girisservo, servoDegeri)
    }
    //% subcategory=MainSensor
    //% block="read motion detection(PIR) module pin %entrypir value"
    export function pir(entrypir: DigitalPin) {
            return pins.digitalReadPin(entrypir);
    }
    //% subcategory=MainSensor
    //% block="read tilt and impact module pin %entrytilt value"
    export function tilt(entrytilt: DigitalPin) {
            return pins.digitalReadPin(entrytilt);
    }
    //% subcategory=MainSensor
    //% block="read potentiometer module pin %entrypot value"
    export function pot(entrypot: AnalogPin) {
            return pins.analogReadPin(entrypot);
    }
    //% subcategory=MainSensor
    //% block="read joystick module x pin %entryjoyx value"
    export function joystickx(entryjoyx: AnalogPin) {
            return pins.analogReadPin(entryjoyx);
    }
    //% subcategory=MainSensor
    //% block="read joystick module y pin %entryjoyy value"
    export function joysticky(entryjoyy: AnalogPin) {
            return pins.analogReadPin(entryjoyy);
    }
    //% subcategory=MainSensor
    //% block="read soil moisture module pin %entrytvn value"
    export function tvn(entrytvn: AnalogPin) {
            return pins.analogReadPin(entrytvn);
    }
    //% subcategory=MainSensor
    //% block="read water level module pin %entrywater value"
    export function water(entrywater: AnalogPin) {
            return pins.analogReadPin(entrywater);
    }
    //% subcategory=MainSensor
    //% block="read ldr module pin %entryldr value"
    export function isik(entryldr: AnalogPin) {
            return pins.analogReadPin(entryldr);
    }
    
}
