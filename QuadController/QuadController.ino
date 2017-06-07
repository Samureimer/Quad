
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define DoDebug

struct Command
{
	float Roll, Pitch, Power;
	bool DoSendStatusreport;
};

struct Status
{
	//float ActualRoll, ActualPitch;
	float MotorFR, MotorFL, MotorRR, MotorRL;
	float Power, PowerPitch, PowerRoll;
	//float BatteryLimitationFactor;
	float BatteryVoltage;
};


Command CurrentCommand;
Status LatestStatus;

RF24 radio(11, 12);
const uint64_t WritingPipe = 0xA8A8E1F0C6LL;
const uint64_t ReadingPipe = 0xA8A8E1F0D7LL;

uint8_t ReadPipeNr = 1; // Which pipe to send on.

void setup(void)
{

	Serial.begin(115200);
	printf_begin();

#pragma region Radio setup
	radio.begin();
	//Receiver.enableAckPayload();
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(70);
	radio.openWritingPipe(WritingPipe);
	radio.openReadingPipe(ReadPipeNr, ReadingPipe);
	radio.startListening();

#ifdef DoDebug
	radio.printDetails();
#endif // DoDebug

#pragma endregion
}

void loop(void)
{
	UpdateCommand();
	Transmit(&CurrentCommand, sizeof(Command));
	if (radio.available(&ReadPipeNr))
	{
		radio.read(&LatestStatus, sizeof(Status));
		//Serial.print("Pitch");
		//Serial.print("\t");
		//Serial.print("Roll");
		//Serial.print("\t");
		Serial.print("FL");
		Serial.print("\t");
		Serial.print("FR");
		Serial.print("\t");
		Serial.print("RL");
		Serial.print("\t");
		Serial.print("RR");
		Serial.print("\t");
		//Serial.print("Bat");
		//Serial.print("\t");
		Serial.println("BatV");

		//Serial.print(LatestStatus.ActualPitch);
		//Serial.print("\t");
		//Serial.print(LatestStatus.ActualRoll);
		//Serial.print("\t");
		Serial.print(LatestStatus.MotorFL);
		Serial.print("\t");
		Serial.print(LatestStatus.MotorFR);
		Serial.print("\t");
		Serial.print(LatestStatus.MotorRL);
		Serial.print("\t");
		Serial.print(LatestStatus.MotorRR);
		Serial.print("\t");
		//Serial.print(LatestStatus.BatteryLimitationFactor);
		//Serial.print("\t");
		Serial.println(LatestStatus.BatteryVoltage);
	}
	delay(500);
}


void UpdateCommand()
{
	CurrentCommand.Power = 50.0;
	CurrentCommand.Pitch = 0.0;
	CurrentCommand.Roll = 0.0;
	CurrentCommand.DoSendStatusreport = true;
}

bool res;
bool Transmit(const void* buf, uint8_t len)
{
	// First, stop listening so we can talk.
	radio.stopListening();
	res = radio.write(buf, len);
	// Now, continue listening
	radio.startListening();

#ifdef DoDebug
	if (res == false)
	{
		Serial.println(F("Failed to send command!"));
	}
	else
	{
		Serial.println(F("Send complete!"));
	}
#endif // DoDebug
	return res;
}
