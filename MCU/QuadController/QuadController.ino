
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "Structs.h"

//#pragma pack(2)

#define DoDebug

//struct Command
//{
//	float Roll, Pitch, Power;
//	bool DoSendStatusreport;
//};
//
//struct Status
//{
//	//float ActualRoll, ActualPitch;
//	float MotorFR, MotorFL, MotorRR, MotorRL;
//	float Power, PowerPitch, PowerRoll;
//	//float BatteryLimitationFactor;
//	float BatteryVoltage;
//};


Command CurrentCommand;

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
	//radio.enableDynamicPayloads();
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
		byte buff[32];

		radio.read(buff, sizeof(buff));
		//for (size_t i = 0; i < sizeof(buff); i++)
		//{
		//	Serial.print(buff[i], DEC);
		//	Serial.print(" ");
		//}
		//Serial.println("");


		//byte status = 3;
		//radio.read(&status, 1);
		//Serial.println(status);
		switch (buff[0])
		{
		case Motor:
		{
			MotorStatus motor;
		}
		break;
		case AngleAndCorrection:
		{
			AngleAndCorrectionStatus angleAndCorrection;
		}
		break;
		case Battery:
		{
			//byte buff[20];
			//for (size_t i = 0; i < sizeof(buff); i++)
			//{
			//	buff[i] = 0;
			//}

			//radio.read(&buff, sizeof(batteryStatus) - 1);
			//for (size_t i = 0; i < sizeof(buff); i++)
			//{
			//	Serial.print(buff[i], DEC);
			//	Serial.print(" ");
			//}
			//Serial.println("");
			//Serial.println((uint16_t)&batteryStatus);
			//Serial.println((uint16_t)((&batteryStatus) + 1));
			//byte* ptr = reinterpret_cast<byte*>(&batteryStatus);

			//memcpy(ptr, &buff, sizeof(batteryStatus));
			//memcpy(&batteryStatus.type, &buff, 1);
			//memcpy(&batteryStatus.BatteryLimitationFactor, &buff + 1, 4);
			//memcpy(&batteryStatus.BatteryVoltage, &buff + 1 + 4, 4);

			BatteryStatus *batteryStatus;
			batteryStatus = (BatteryStatus*)buff;

			Serial.print("Voltage: ");
			Serial.println(batteryStatus->BatteryVoltage);
			Serial.println(batteryStatus->BatteryLimitationFactor);
			for (size_t i = 0; i < sizeof(BatteryStatus); i++)
			{
				Serial.print((((uint8_t*)&batteryStatus)[i]));
				Serial.print(' ');
			}
			Serial.println();
		}
		break;
		}
	}
	delay(500);
}


void UpdateCommand()
{
	CurrentCommand.Power = 50.0;
	CurrentCommand.Pitch = 0.0;
	CurrentCommand.Roll = 0.0;
	CurrentCommand.StatusToSend = Battery;
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
