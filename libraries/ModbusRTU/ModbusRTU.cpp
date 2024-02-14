
//==============================================================================
//                              ModbusRTU.cpp
//==============================================================================

/*-------------------------------------------------------------------------------
                           MODBUS RTU COMMUNICATION

    Description:
       Partial implementation of the Modbus RTU communication protocol.

    Implemented functions:
       1) Function 03 (03 hex): Read Holding Registers
       2) Function 06 (06 hex): Write Single Register
       3) Function 16 (10 hex): Write Multiple Registers

    REFERENCES:
      1) http://www.modbustools.com/modbus.html
      2) http://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf


-------------------------------------------------------------------------------*/


#include "modbusRTU.h"


/*==============================================================================
Function    : CRC_16
Description : Performs the calculation of CRC16.
Parameters  : nData  : Pointer to the data on which the CRC16 will be calculated;
              wLength: Amount of data (in bytes) that will be processed in calculation.
Return      : Calculated CRC value.
=========================================================== ===================*/
unsigned int CRC_16(const unsigned char *nData, unsigned int wLength) {
   unsigned char nTemp;
   unsigned int wCRCWord = 0xFFFF;

   while (wLength--) {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord  ^= wCRCTable[nTemp];
   }
   return wCRCWord;
}


/*==============================================================================
Function    : ModbusRTU_Pack
Description : Performs the interpretation of received Modbus RTU messages and 
              also the Modbus RTU messages to be sent.
Parameters  : MBBuff: Pointer to original buffer where is original message;
              MBHoldingReg: Pointer to the Modbus register array.
Return      : > 0: SUCCESS = Message size to be transmitted;
              = 0: Error or no message to process as a response.
==============================================================================*/
unsigned char ModbusRTU_Pack(unsigned char *MBBuff, unsigned int *MBHoldingReg) {

   unsigned int DataByteLength;
   unsigned int RegisterStart;
   unsigned int RegistersCount;
   unsigned int MessageLength;
   unsigned int CRC_Ref;

   unsigned char DeviceAddr;
	unsigned char MB_Function;


	// Selects Request message parameters
   DeviceAddr     = MBBuff[MB_RTU_DEV_ADDR];
   MB_Function    = MBBuff[MB_RTU_FUNC];
   RegisterStart  = (MBBuff[MB_RTU_REGISTER_START] << 8) + MBBuff[MB_RTU_REGISTER_START +1]; 
   RegistersCount = (MBBuff[MB_RTU_REGISTER_COUNT] << 8) + MBBuff[MB_RTU_REGISTER_COUNT +1];

	// The devices start from the value "1"
	if (DeviceAddr == 0) { return 0; }
   if (RegistersCount > MB_MAX_HOLDING_REGISTER) { return 0; }

   switch(MB_Function) {
      case MB_FC_NONE:
         MessageLength = 0;
         break;
      
      case MB_FC_READ_REGISTERS: // 03 Read Holding Registers

         // Checks whether the request message CRC is valid 
         CRC_Ref = (MBBuff[MB_RTU_CRC +1] << 8) + MBBuff[MB_RTU_CRC];

         if (CRC_Ref != CRC_16(MBBuff, 6)) { return 0; }

         // Packages response to the current request
         DataByteLength             = RegistersCount * 2;
         MBBuff[MB_RTU_DEV_ADDR]    = DeviceAddr;
         MBBuff[MB_RTU_FUNC]        = MB_Function;
         MBBuff[MB_RTU_BYTE_COUNT]  = DataByteLength;
         
         // Assigns in the response the current values of the registers
         for (int i = 0; i < RegistersCount; i++) {
            MBBuff[ MB_RTU_RESPONSE_DATA      + i * 2] = (MBHoldingReg[RegisterStart + i] >> 8) & 0xFF;
            MBBuff[(MB_RTU_RESPONSE_DATA + 1) + i * 2] = MBHoldingReg[RegisterStart + i] & 0xFF;
         }

         // Calculates the CRC16
         #define HEADER_DATA_READ 3 // "3" = 3 header bytes
         CRC_Ref = CRC_16(MBBuff, HEADER_DATA_READ + DataByteLength); 
         MBBuff[HEADER_DATA_READ + DataByteLength]    = CRC_Ref & 0xFF;
         MBBuff[HEADER_DATA_READ + DataByteLength +1] = (CRC_Ref >> 8) & 0xFF;

         MessageLength = HEADER_DATA_READ + DataByteLength + MB_RTU_CRC_LEN; 
         break;
      
      
      case MB_FC_WRITE_REGISTER: // 06 Write Holding Register

         // Checks whether the request message CRC is valid
         CRC_Ref = (MBBuff[MB_RTU_CRC +1] << 8) + MBBuff[MB_RTU_CRC];

         if (CRC_Ref != CRC_16(MBBuff, 6)) { return 0; }

         // Assigns the new value to register
         MBHoldingReg[RegisterStart] = (MBBuff[MB_RTU_S_WRITE_DATA_START] << 8) + MBBuff[MB_RTU_S_WRITE_DATA_START +1];

         // Response: Response is an echo of the request, which always has size "8"
         MessageLength = 8;
         break;

      
      case MB_FC_WRITE_MULTIPLE_REGISTERS: // 16 Write Holding Registers

         // Checks whether the request message CRC is valid
         CRC_Ref = (MBBuff[MB_RTU_REGISTER_COUNT + 2 + 1 + RegistersCount * 2 + 1] << 8) +
                    MBBuff[MB_RTU_REGISTER_COUNT + 2 + 1 + RegistersCount * 2]; 

         if (CRC_Ref != CRC_16(MBBuff, MB_RTU_REGISTER_COUNT + 2 + 1 + RegistersCount * 2)) { return 0; }

         // Assigns the registers with the new values received in the request
         for (int i = 0; i < RegistersCount; i++)
            MBHoldingReg[RegisterStart + i] = (MBBuff[MB_RTU_M_WRITE_DATA_START + i * 2] << 8) + MBBuff[(MB_RTU_M_WRITE_DATA_START +1) + i * 2];

         // Packages response to the current request 
         MBBuff[MB_RTU_DEV_ADDR]          = DeviceAddr;
         MBBuff[MB_RTU_FUNC]              = MB_Function;
         MBBuff[MB_RTU_REGISTER_START]    = (RegisterStart >> 8) & 0xFF;
         MBBuff[MB_RTU_REGISTER_START +1] = RegisterStart & 0xFF;
         MBBuff[MB_RTU_REGISTER_COUNT]    = (RegistersCount >> 8) & 0xFF;
         MBBuff[MB_RTU_REGISTER_COUNT +1] = RegistersCount & 0xFF;

         // Calculates the CRC16
         #define WRITE_ANS_LENGTH 6
         CRC_Ref = CRC_16(MBBuff, WRITE_ANS_LENGTH);
         MBBuff[WRITE_ANS_LENGTH]    = CRC_Ref & 0xFF;
         MBBuff[WRITE_ANS_LENGTH +1] = (CRC_Ref >> 8) & 0xFF;

         // The length of the response is always size "8"
         MessageLength = 8;
         break;


      default:
         // If no Modbus function has been processed, then it only returns "0" to 
         // indicate there is nothing to send via Serial Port.
         MessageLength = 0;
         break;
   }

   return MessageLength;
}




