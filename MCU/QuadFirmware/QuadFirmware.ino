

#include <SPI.h>
#include <nRF24L01-STM.h>
#include <RF24-STM.h>
#include "MotorPid.h"
#include "Structs.h"
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

#define DoDebug

#define INTERRUPT_PIN PB8
#define LED_PIN PC13
#define ReadPipeNr 1
#define CE PC14
#define CS PC15
#define MotorFRPin PA10
#define MotorFLPin PB0
#define MotorRRPin PA9
#define MotorRLPin PB1
#define BatteryPin PA1
#define TransmitTimeout 75


MPU6050 mpu;
MotorPID RollPid;
MotorPID PitchPid;

RF24 Receiver(CE, CS);
const uint64_t ReadingPipe = 0xA8A8E1F0C6LL;
const uint64_t WritingPipe = 0xA8A8E1F0D7LL;
Command CurrentCommand;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

						// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


float MotorFR, MotorFL, MotorRR, MotorRL;
float PowerPitch, PowerRoll;
float BatteryVoltage;

bool DoOffsetCalc = true;
bool DoSkipForCalc = true;
const uint16_t FifosToSkip = 500;
uint16_t FifosSkiped = 0;

bool CalculatingRoll = true;
const uint8_t OffsetsCalculationsToDo = 10;
uint8_t OffsetsCalculated = 0;
float OffsetsCalculations[OffsetsCalculationsToDo];
float BatteryLimitationFactor = 0.0f;

bool StartedTransmit = false;
uint32 TransmitStartTime = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
	mpuInterrupt = true;
}

void setup() {
	//CurrentCommand.Power = 80;

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, true);


#pragma region PWM setup
	timer_dev *dev = PIN_MAP[MotorFRPin].timer_device;
	timer_set_prescaler(dev, 0);
	timer_generate_update(dev);

	dev = PIN_MAP[MotorFLPin].timer_device;
	timer_set_prescaler(dev, 0);
	timer_generate_update(dev);

	dev = PIN_MAP[MotorRRPin].timer_device;
	timer_set_prescaler(dev, 0);
	timer_generate_update(dev);

	dev = PIN_MAP[MotorRLPin].timer_device;
	timer_set_prescaler(dev, 0);
	timer_generate_update(dev);

	pinMode(MotorFRPin, PWM);
	pinMode(MotorFLPin, PWM);
	pinMode(MotorRRPin, PWM);
	pinMode(MotorRLPin, PWM);
	analogWrite(MotorFRPin, 255);
	analogWrite(MotorFLPin, 255);
	analogWrite(MotorRRPin, 255);
	analogWrite(MotorRLPin, 255);
#pragma endregion

#pragma region Serial setup
#ifdef DoDebug
	Serial.begin(115200);
	while (!Serial);
#endif //DoDebug
#pragma endregion

#pragma region MPU setup
	Wire.begin();
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	//mpu.setXGyroOffset(220);
	//mpu.setYGyroOffset(76);
	//mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	if (devStatus == 0) {
		mpu.setDMPEnabled(true);
		pinMode(INTERRUPT_PIN, INPUT);
		attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
#pragma endregion

#pragma region PID setup
	PitchPid.Setup(10.0f, 0.0005f, 0.005f);
	RollPid.Setup(10.0f, 0.0005f, 0.005f);
#pragma endregion

#pragma region Radio setup
	Receiver.begin();
	Receiver.setRetries(2, 5);
	//Receiver.enableAckPayload();
	Receiver.setDataRate(RF24_250KBPS);
	Receiver.setChannel(70);
	Receiver.openWritingPipe(WritingPipe);
	Receiver.openReadingPipe(ReadPipeNr, ReadingPipe);
	Receiver.startListening();
#pragma endregion

	while (dmpReady == false)
	{
		Serial.println("DMP not ready.");
		delay(500);
	}


	digitalWrite(LED_PIN, false);
	delay(1000);
}

float Roll, Pitch;
float RollOffset, PitchOffset;
void loop()
{
	Serial.println(CurrentCommand.Power);
	Serial.print(BatteryVoltage);
	Serial.println(" V");
	PrintDebugInfo();

	if (Serial.available() > 0)
	{
		CurrentCommand.Power = Serial.parseInt();
		//char a = Serial.read();
		//analogWrite(MotorFRPin, 255);
		//analogWrite(MotorFLPin, 255);
		//analogWrite(MotorRRPin, 255);
		//analogWrite(MotorRLPin, 255);

		//switch (a)
		//{
		//case 'P':
		//	CurrentCommand.Power = Serial.parseInt();
		//	break;
		//case '1':
		//	analogWrite(MotorFRPin, 255 - CurrentCommand.Power);
		//	break;
		//case '2':
		//	analogWrite(MotorFLPin, 255 - CurrentCommand.Power);
		//	break;
		//case '3':
		//	analogWrite(MotorRRPin, 255 - CurrentCommand.Power);
		//	break;
		//case '4':
		//	analogWrite(MotorRLPin, 255 - CurrentCommand.Power);
		//	break;
		//default:
		//	break;
		//}
	}

	if (mpuInterrupt || fifoCount > packetSize)
	{
		ProcessMpuData();

		if (DoOffsetCalc)
		{
			digitalWrite(LED_PIN, true);
			ProcessOffsetCalculations();
		}
		else
		{
			ProcessPID();
			ProcessMotorPower();
		}
	}
	//else
	//{
	//	ProcessReceiver();
	//}

	ProcessReceiver();
	ProcessBatteryVoltage();
}

void ProcessBatteryVoltage() {
	BatteryVoltage = analogRead(BatteryPin) / 4095.0f * 5.0f;
	BatteryLimitationFactor = (BatteryVoltage - 3.1f) / 0.3f;
	//Serial.println(BatteryVoltage);
	//BatteryLimitationFactor = constrain(BatteryLimitationFactor, 0.0f, 1.0f);
}

void ProcessMotorPower()
{
	MotorFL = (255 - PowerPitch - PowerRoll) * CurrentCommand.Power / 255;
	//MotorFL = CurrentCommand.Power - PowerPitch - PowerRoll;

	MotorFR = (255 - PowerPitch + PowerRoll) * CurrentCommand.Power / 255;
	//MotorFR = CurrentCommand.Power - PowerPitch + PowerRoll;


	MotorRL = (PowerPitch - PowerRoll) * CurrentCommand.Power / 255;
	//MotorRL = CurrentCommand.Power + PowerPitch - PowerRoll;

	MotorRR = (PowerPitch + PowerRoll) * CurrentCommand.Power / 255;
	//MotorRR = CurrentCommand.Power + PowerPitch + PowerRoll;


	MotorFL = constrain(MotorFL, 0.0f, 255.0f) * BatteryLimitationFactor;
	//MotorFL = MotorFL > 255.0 ? 255.0 : MotorFL;
	//MotorFL = MotorFL < 0.0 ? 0.0 : MotorFL;

	MotorFR = constrain(MotorFR, 0.0f, 255.0f) * BatteryLimitationFactor;
	//MotorFR = MotorFR > 255.0 ? 255.0 : MotorFR;
	//MotorFR = MotorFR < 0.0 ? 0.0 : MotorFR;

	MotorRL = constrain(MotorRL, 0.0f, 255.0f) * BatteryLimitationFactor;
	//MotorRL = MotorRL > 255.0 ? 255.0 : MotorRL;
	//MotorRL = MotorRL < 0.0 ? 0.0 : MotorRL;

	MotorRR = constrain(MotorRR, 0.0f, 255.0f) * BatteryLimitationFactor;
	//MotorRR = MotorRR > 255.0 ? 255.0 : MotorRR;
	//MotorRR = MotorRR < 0.0 ? 0.0 : MotorRR;

	analogWrite(MotorFRPin, 255 - MotorFL);
	analogWrite(MotorFLPin, 255 - MotorFR);
	analogWrite(MotorRRPin, 255 - MotorRL);
	analogWrite(MotorRLPin, 255 - MotorRR);

	//Serial.println(255 - CurrentCommand.Power);
	//analogWrite(MotorFRPin, 255 - CurrentCommand.Power);
	//analogWrite(MotorFLPin, 255 - CurrentCommand.Power);
	//analogWrite(MotorRRPin, 255 - CurrentCommand.Power);
	//analogWrite(MotorRLPin, 255 - CurrentCommand.Power);
}

void ProcessOffsetCalculations()
{
	if (DoSkipForCalc)
	{
		FifosSkiped++;
		if (FifosSkiped == FifosToSkip)
			DoSkipForCalc = false;
	}
	else
	{
		if (CalculatingRoll == true)
			OffsetsCalculations[OffsetsCalculated] = Roll;
		else
			OffsetsCalculations[OffsetsCalculated] = Pitch;

		OffsetsCalculated++;

		if (OffsetsCalculated == OffsetsCalculationsToDo && CalculatingRoll == true)
		{
			CalculatingRoll = false;
			OffsetsCalculated = 0;

			float offset = 0;
			for (uint8_t i = 0; i < OffsetsCalculationsToDo; i++)
				offset += OffsetsCalculations[i];

			RollOffset = offset / OffsetsCalculationsToDo;
		}
		else if (OffsetsCalculated == OffsetsCalculationsToDo && CalculatingRoll == false)
		{
			float offset = 0;
			for (uint8_t i = 0; i < OffsetsCalculationsToDo; i++)
				offset += OffsetsCalculations[i];

			PitchOffset = offset / OffsetsCalculationsToDo;
			DoOffsetCalc = false;

			digitalWrite(LED_PIN, false);
		}
	}
}

void ProcessPID()
{
	PowerRoll += RollPid.Process(CurrentCommand.Roll, Roll);
	if (PowerRoll > 255)
		PowerRoll = 255;
	else if (PowerRoll < -255)
		PowerRoll = -255;

	PowerPitch += PitchPid.Process(CurrentCommand.Pitch, Pitch);
	if (PowerPitch > 255)
		PowerPitch = 255;
	else if (PowerPitch < -255)
		PowerPitch = -255;
}

BatteryStatus batteryStatus;
void ProcessReceiver() {

	uint8_t pipe;

	if (StartedTransmit)
	{
		bool tx_ok;
		bool tx_fail;
		bool rx_ready;

		Receiver.whatHappened(tx_ok, tx_fail, rx_ready);

		if (tx_ok || tx_fail || rx_ready || millis() - TransmitStartTime > TransmitTimeout)
		{
			Serial.print("OK: ");
			Serial.print(tx_ok);
			Serial.print("\t");
			Serial.print("Fail: ");
			Serial.print(tx_fail);
			Serial.print("\t");
			Serial.print("RX: ");
			Serial.println(rx_ready);

			Receiver.startListening();
			StartedTransmit = false;
		}
	}

	if (StartedTransmit == true)
		return;

	if (Receiver.available(&pipe))
	{
		if (pipe == ReadPipeNr)
		{
			Receiver.read(&CurrentCommand, sizeof(Command));
		}
	}


	switch (CurrentCommand.StatusToSend)
	{
	case Motor:
	{
		MotorStatus motor;
		GenerateMotorStatus(&motor);
		TransmitAsync(&motor, sizeof(MotorStatus));
	}
	break;
	case AngleAndCorrection:
	{
		AngleAndCorrectionStatus angleAndCorrection;
		GenerateAngleAndCorrectionStatus(&angleAndCorrection);
		TransmitAsync(&angleAndCorrection, sizeof(AngleAndCorrectionStatus));
	}
	break;
	case Battery:
	{
		GenerateBatteryStatus(&batteryStatus);
		Serial.print("Voltage: ");
		Serial.println(batteryStatus.BatteryVoltage);
		TransmitAsync(&batteryStatus, sizeof(BatteryStatus));
		sizeof(BatteryStatus);
	}
	break;
	}
	CurrentCommand.StatusToSend = None;
}

void GenerateAngleAndCorrectionStatus(AngleAndCorrectionStatus *status) {
	status->Roll = Roll;
	status->Pitch = Pitch;
	status->PowerPitch = PowerPitch;
	status->PowerRoll = PowerRoll;
}

void GenerateMotorStatus(MotorStatus *status) {
	status->MotorFL = MotorFL;
	status->MotorFR = MotorFR;
	status->MotorRL = MotorRL;
	status->MotorRR = MotorRR;
	status->Power = CurrentCommand.Power;
}

void GenerateBatteryStatus(BatteryStatus *status) {
	status->BatteryLimitationFactor = 123.0f;
	status->BatteryVoltage = BatteryVoltage;
	Serial.println(status->BatteryVoltage);
}

void ProcessMpuData() {
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
	}
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


		//Pitch
		Roll = ypr[1] * 180.0f / M_PI;
		if (gravity.z <= 0 && gravity.x <= 0)
			Roll = -180 - Roll;
		else if (gravity.z <= 0)
			Roll = 180 - Roll;

		//Roll
		Pitch = ypr[2] * 180.0f / M_PI;
		if (gravity.z <= 0 && gravity.y <= 0)
			Pitch = -180 - Pitch;
		else if (gravity.z <= 0)
			Pitch = 180 - Pitch;

		Roll -= RollOffset;
		Roll = -Roll;

		Pitch -= PitchOffset;
		Pitch = -Pitch;
	}
}

bool Transmit(const void* buf, uint8_t len)
{
	Receiver.stopListening();
	bool res = Receiver.write(buf, len);
	Receiver.startListening();
	return res;
}

void TransmitAsync(const void* buf, uint8_t len)
{
	TransmitStartTime = millis();
	StartedTransmit = true;
	Receiver.stopListening();
	Receiver.startWrite(buf, len);
}

void PrintDebugInfo()
{
#ifdef DoDebug
	//BatteryStatus batState;
	//GenerateBatteryStatus(&batState);
	//Serial.print("Bat V: ");
	//Serial.println(batState.BatteryVoltage);
	//Serial.print("Bat lim: ");
	//Serial.println(batState.BatteryLimitationFactor);

	AngleAndCorrectionStatus AGStatus;
	GenerateAngleAndCorrectionStatus(&AGStatus);
	Serial.print("Roll: ");
	Serial.print(AGStatus.Roll);
	Serial.print("\tPitch: ");
	Serial.print(AGStatus.Pitch);
	Serial.print("\tPower Roll: ");
	Serial.print(AGStatus.PowerRoll);
	Serial.print("\tPower Pitch: ");
	Serial.println(AGStatus.PowerPitch);

#endif // DoDebug
}