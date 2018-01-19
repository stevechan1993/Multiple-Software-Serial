/*
  Software serial multple serial test

 Receives from the two software serial ports,
 sends to the hardware serial port.

 In order to listen on a software port, you call port.listen().
 When using two software serial ports, you have to switch ports
 by listen()ing on each one in turn. Pick a logical time to switch
 ports, like the end of an expected transmission, or when the
 buffer is empty. This example switches ports when there is nothing
 more to read from a port

 The circuit:
 Two devices which communicate serially are needed.
 * First serial device's TX attached to digital pin 10(RX), RX to pin 11(TX)
 * Second serial device's TX attached to digital pin 8(RX), RX to pin 9(TX)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created 18 Apr. 2011
 modified 19 March 2016
 by Tom Igoe
 based on Mikal Hart's twoPortRXExample

 This example code is in the public domain.

 */

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <JY901.h>


long lat, lon;

TinyGPS gps;

// software serial #2: RX = digital pin 8, TX = digital pin 9
// on the Mega, use other pins instead, since 8 and 9 don't work on the Mega
// This software serial used to get IMU data
SoftwareSerial imuSerial(8, 9);

// software serial #1: RX = digital pin 10, TX = digital pin 11
// This software serial used to get GPS data
SoftwareSerial gpsSerial(10, 11);

// software serial #1: RX = digital pin 12, TX = digital pin 13
// This software serial used to get Megnetic data
SoftwareSerial magSerial(12, 13);

// 定义加速度，角速度，角度，温度
float A[3], W[3], ANGLE[3], T;

// 定义数据缓存区
unsigned char Re_buf[11];

// 定义数据帧读取计数器
unsigned char counter = 0;

// Toggle reading data flag
unsigned char flag = 1;

unsigned char cnt = 0;

unsigned char limit = 4;

unsigned char i = 0, j = 0, k = 0, l = 0;

// 磁力计角度获取
float magAngleZ, magAngleX, magAngleY;

String LatitudeStr, LongitudeStr;

// Device Info Defination
typedef struct DEVICE_INFO {
	String longitude;
	String latitude;
	double angleforNorth;
	double angleforVertical;
	int deviceid;
};

char Uart_Buffer;
char GPS_COM[6];
char RX_GPGGA_Buffer[80];
char RX_GPRMC_Buffer[80];
unsigned char RX_Count;
bool Flag_GPGGA_OK;
bool Flag_GPRMC_OK;
bool Flag_GPS_OK;
unsigned char UTC_Hour;
unsigned char UTC_Min;
unsigned char UTC_Sec;

String strLon, strLat;

char Altitude[4];

// Device Info 结构体
DEVICE_INFO deviceInfo;

void toggleReadData() {
	// toggle reading multiple sensor data
	switch (cnt)
	{
	//case 0:   // reading imu data
	//	//Serial.println("Data from port IMU:");
	//	gpsSerial.end();
	//	magSerial.end();
	//	imuSerial.begin(115200);
	//	break;
	case 1:   // reading gps data
		//Serial.println("Data from port GPS:");
		imuSerial.end();
		magSerial.end();
		gpsSerial.begin(9600);
		break;
	case 2:   // reading megnetic data
		//Serial.println("Data from port MEGNETIC:");
		imuSerial.end();
		gpsSerial.end();
		magSerial.begin(9600);

		magAngleX = (float)JY901.stcAngle.Angle[0] / 32768 * 180;

		magAngleY = (float)JY901.stcAngle.Angle[1] / 32768 * 180;

		magAngleZ = (float)JY901.stcAngle.Angle[2] / 32768 * 180;

		/*Serial.println(magAngleX);
		Serial.println(magAngleY);
		Serial.println(magAngleZ);   */

		deviceInfo.angleforNorth = magAngleZ;
		deviceInfo.angleforVertical = magAngleY;

		// 输出设备信息
		//printDeviceInfo();
		break;
	case 3: 
		imuSerial.end();
		magSerial.end();
		gpsSerial.end();
		Serial.begin(9600);
	default:
		break;
	}

	cnt++;

	if (cnt == limit) {   // clear the counter
		cnt = 0;
	}
}

// 数组指定位置插入字符
void insertData(char arr[], int len, int loc, char val)
{
	int i = len;
	while (i>loc)
	{
		arr[i] = arr[i - 1];
		i--;
	}
	arr[i] = val;
	len++;
}

// 输出设备数据
void printDeviceInfo() {
	// Send the data to hardware port
	//Serial.print("Device info:"); 
	Serial.print(deviceInfo.longitude); 
	Serial.print('|');
	Serial.print(deviceInfo.latitude); 
	Serial.print('|');
	Serial.print(deviceInfo.angleforNorth); 
	Serial.print('|');
	Serial.print(deviceInfo.angleforVertical); 
	Serial.print('|');
	Serial.print(deviceInfo.deviceid);
	Serial.println();
}

void parseIMU() {
	// Now listen on the second port
	//imuSerial.listen();
	// while there is data coming in, read it
	// and send to the hardware serial port:
	//Serial.println("Data from port IMU:");
	if (imuSerial.available() > 0) {
		char inByte = imuSerial.read();
		//Serial.print(inByte);

		// 按字节读取惯导数据
		Re_buf[counter] = inByte;

		// 只读取包头为0X55的数据包，其他数据丢弃
		if (counter == 0 && Re_buf[0] != 0x55) return;

		counter++;

		// 完成一帧数据的读取后，清除计数器
		if (counter == 11) {
			counter = 0;

			if (Re_buf[0] == 0x55) {
				switch (Re_buf[1]) {
				case 0x51:   //加速度计算
					A[0] = (short(Re_buf[3] << 8 | Re_buf[2])) / 32768.0 * 16;
					A[1] = (short(Re_buf[5] << 8 | Re_buf[4])) / 32768.0 * 16;
					A[2] = (short(Re_buf[7] << 8 | Re_buf[6])) / 32768.0 * 16;

					//计算温度
					T = (short(Re_buf[9] << 8 | Re_buf[8])) / 340.0 + 36.25;

					//输出加速度到硬件串口
					/*Serial.print("A: ");
					Serial.print(A[0]);
					Serial.print(",");
					Serial.print(A[1]);
					Serial.print(",");
					Serial.print(A[2]);
					Serial.println();	*/
					break;
				case 0x52:   //角速度计算
					W[0] = (short(Re_buf[3] << 8 | Re_buf[2])) / 32768.0 * 2000;
					W[1] = (short(Re_buf[5] << 8 | Re_buf[4])) / 32768.0 * 2000;
					W[2] = (short(Re_buf[7] << 8 | Re_buf[6])) / 32768.0 * 2000;

					// 计算温度
					T = (short(Re_buf[9] << 8 | Re_buf[8])) / 340.0 + 36.25;

					//输出角速度到硬件串口
					/*Serial.print("W: ");
					Serial.print(W[0]);
					Serial.print(",");
					Serial.print(W[1]);
					Serial.print(",");
					Serial.print(W[2]);
					Serial.println();	*/
					break;
				case 0x53:   //角度计算
					ANGLE[0] = (short(Re_buf[3] << 8 | Re_buf[2])) / 32768.0 * 180;
					ANGLE[1] = (short(Re_buf[5] << 8 | Re_buf[4])) / 32768.0 * 180;
					ANGLE[2] = (short(Re_buf[7] << 8 | Re_buf[6])) / 32768.0 * 180;

					// 温度计算
					T = (short(Re_buf[9] << 8 | Re_buf[8])) / 340.0 + 36.25;

					//输出角度到硬件串口
					/*Serial.print("Angle: ");
					Serial.print(ANGLE[0]);
					Serial.print(",");
					Serial.print(ANGLE[1]);
					Serial.print(",");
					Serial.print(ANGLE[2]);
					Serial.println();*/

					deviceInfo.angleforVertical = ANGLE[0];
					deviceInfo.angleforNorth = ANGLE[2];
					break;
				}

				// 输出设备信息
				//printDeviceInfo();
			}
		}
	}
}

void parseMAG() {
	// blank line to separate data from the two ports:
	//Serial.println();

	// Now listen on the third port
	//portThree.listen();
	// while there is data coming in, read it
	// and send to the hardware serial port:
	//Serial.println("Data from port three:");
	while (magSerial.available() > 0) {
		char inByte = magSerial.read();
		//Serial.write(inByte);

		JY901.CopeSerialData(inByte);   // Call JY901 data cope function

		//magAngleX = (float)JY901.stcAngle.Angle[0] / 32768 * 180;

		//magAngleY = (float)JY901.stcAngle.Angle[1] / 32768 * 180;

		//magAngleZ = (float)JY901.stcAngle.Angle[2] / 32768 * 180;

		//Serial.println(magAngleX);
		//Serial.println(magAngleY);
		//Serial.println(magAngleZ);

		//deviceInfo.angleforNorth = magAngleZ;
		//deviceInfo.angleforVertical = magAngleY;

		//// 输出设备信息
		//printDeviceInfo();

		//// blank line to separate data from the three ports:
		////Serial.println();
	}
}

// Parse the GPS data (NAME)
void parseGPS() {
	char Longitude[10];
	char Latitude[9];

	while (gpsSerial.available() > 0) {
		Uart_Buffer = gpsSerial.read();

		if (Uart_Buffer == '$') {
			RX_Count = 0;
		}

		if (RX_Count < 6) {
			GPS_COM[RX_Count++] = Uart_Buffer;
		}
		else if (GPS_COM[0] == '$' && GPS_COM[1] == 'G' && GPS_COM[2] == 'N' && GPS_COM[3] == 'G' && GPS_COM[4] == 'G' && GPS_COM[5] == 'A') {    // Get the GPGGA data
			RX_GPGGA_Buffer[RX_Count] = Uart_Buffer;
			if (RX_GPGGA_Buffer[RX_Count] == '\n') {
				Flag_GPGGA_OK = 1;
			}
			else {
				RX_Count++;
			}
		}
		else if (GPS_COM[0] == '$' && GPS_COM[1] == 'G' && GPS_COM[2] == 'N' && GPS_COM[3] == 'R' && GPS_COM[4] == 'M' && GPS_COM[5] == 'C') {     //Get the GPRMC data
			RX_GPRMC_Buffer[RX_Count] = Uart_Buffer;
			if (RX_GPRMC_Buffer[RX_Count] == '\n'){
				Flag_GPRMC_OK = 1;
			}
			else {
				RX_Count++;
			}
		}
		else if (GPS_COM[0] == '$' && GPS_COM[1] == 'G' && GPS_COM[2] == 'N' && GPS_COM[3] == 'G' && GPS_COM[4] == 'L' && GPS_COM[5] == 'L') {	    //Get the GPGLL data
			if (Uart_Buffer == '\n'){
				Flag_GPS_OK = 1;       //一组数据都是最后一帧是GPGLL，接收完GPGLL说明接收数据完成
			}
		}

		if (Flag_GPS_OK == 1)
		{
			Flag_GPS_OK = 0;
			if (Flag_GPGGA_OK == 1)
			{
				Flag_GPGGA_OK = 0;

				//UTC_Hour = (RX_GPGGA_Buffer[7] - 0x30) * 10 + (RX_GPGGA_Buffer[8] - 0x30);   //获取UTC时间
				//UTC_Min = (RX_GPGGA_Buffer[9] - 0x30) * 10 + (RX_GPGGA_Buffer[10] - 0x30);
				//UTC_Sec = (RX_GPGGA_Buffer[11] - 0x30) * 10 + (RX_GPGGA_Buffer[12] - 0x30);

				//Serial.print("UTC_Time: ");

				/*Serial.print(UTC_Hour);
				Serial.print(":");
				Serial.print(UTC_Min);
				Serial.print(":");
				Serial.println(UTC_Sec);*/

				/*Serial.println("Altitude: ");

				for (i = 0; i < 4; i++)
				{
					Altitude[i] = RX_GPGGA_Buffer[54 + i];
					Serial.print(Altitude[i]);
				}

				Serial.println();*/

				//Serial.println("Longitude: ");   // 经度输出

				// 高五位
				for (i = 0; i < 5; i++) {
					Longitude[i] = RX_GPGGA_Buffer[30 + i];
					Serial.print(Longitude[j]);
				}

				// 低五位
				for (j = 0; j < 5; j++) {
					Longitude[j] = RX_GPGGA_Buffer[36 + j];
					Serial.print(Longitude[j]);
				}

				insertData(Longitude, 10, 3, '.');  // 正确的位置补上小数点

				//strLon = String(Longitude);   // turn char[] to string

				//Serial.println(strLon);

				//Serial.println("Latitude: ");    // 纬度输出

				// 高四位
				for (k = 0; k < 4; k++) {
					Latitude[k] = RX_GPGGA_Buffer[17 + k];
					Serial.print(Latitude[k]);
				}

				// 低五位
				for (l = 0; l < 5; l++) {
					Latitude[l] = RX_GPGGA_Buffer[22 + l];
					Serial.print(Latitude[k]);
				}

				insertData(Latitude, 9, 2, '.');    // 正确的位置补上小数点

				//strLat = String(Latitude);    // turn char[] to string

				//Serial.println(strLat);

				// 输出设备信息
				deviceInfo.latitude = Latitude;
				deviceInfo.longitude = Longitude;
				LatitudeStr = Latitude;
				LongitudeStr = Longitude;

				//printDeviceInfo();
			}
		}
	}
}


void setup() {
	// Open serial communications and wait for port to open:
	//Serial.begin(9600);
	//while (!Serial) {
	//	; // wait for serial port to connect. Needed for native USB port only
	//}

	// Start each software serial port
	//imuSerial.begin(115200);
	//gpsSerial.begin(9600);   
	//magSerial.begin(9600);

	Flag_GPGGA_OK = 0;
	RX_Count = 0;
	Uart_Buffer = 0;

	deviceInfo.deviceid = 1;

	MsTimer2::set(1000, toggleReadData);   // set the timer
	MsTimer2::start();    // start the timer
} 

void loop() {
	// By default, the last intialized port is listening.
	// when you want to listen on a port, explicitly select it:
	//gpsSerial.listen();

	// blank line to separate data from the two ports:
	//Serial.println();

	//Get GPS data and parse the data
	parseGPS();

	//Get IMU data and parse the data
	parseIMU();

	//Get MAG data and parse the data
	parseMAG();

	// Now listen on the hardware port
	while (Serial.available() > 0) {
		char inByte = Serial.read();

		if (inByte == 'a') {
			deviceInfo.latitude = LatitudeStr;
			deviceInfo.longitude = LongitudeStr;
			//121.336514,31.245835
			deviceInfo.deviceid = 1;
			deviceInfo.angleforVertical = magAngleY;
			deviceInfo.angleforNorth = magAngleZ;

			printDeviceInfo();
		}
	}
}

/*
SerialEvent occurs whenever a new data comes in the
hardware serial RX.  This routine is run between each
time loop() runs, so using delay inside loop can delay
response.  Multiple bytes of data may be available.
*/
//void serialEvent()
//{
//	while (magSerial.available())
//	{
//		JY901.CopeSerialData(magSerial.read()); //Call JY901 data cope function
//	}
//}
