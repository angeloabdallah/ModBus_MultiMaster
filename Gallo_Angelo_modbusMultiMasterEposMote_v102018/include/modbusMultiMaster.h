#ifndef __cortex_modbusMultiMaster_h__
#define __cortex_modbusMultiMaster_h__

/*
    Purpose:    Modified Version of ModBus RTU Wich Supports Multi-Master (One at a time)
    Reviewed:   October, 2018
    Author:     Angelo Abdallah Pelisson (abdallah96378494@hotmail.com)
                Wellington Rodrigo Gallo (w.r.gallo@grad.ufsc.br)
                Copyright (C) 2018
    License:    Software License Agreement (BSD License)
                This program is a free software; 
                You can:
                   * redistribute it
                   * modify it
                Under the terms of the GNU General Public License
                published by the Free Software Foundation.
    To-do:
                Processing Error and Formatting Error Reply such the cases of wrong CRC, timeout while waiting response, ...
*/

#include "uart.h"
#include "gpio.h"
#include "machine.h"
#include "cpu.h"
#include <alarm.h>
#include <timer.h>
#include <rs485.h>
#include <ic.h>

__BEGIN_SYS

typedef RS485 _rs485;

class modbusMultiMaster : private _rs485
{
    enum MODBUS_BAUDRATE {
        BAUDRATE_9600               = 0x00,
        BAUDRATE_19200              = 0x01
    };

    enum Commands {
        READ_COILS                  = 0x01,
        READ_HOLDING_REGISTER       = 0x03,
        WRITE_SINGLE_COIL           = 0x05,
        WRITE_SINGLE_REGISTER       = 0x06,
        WRITE_MULTIPLE_COILS        = 0x15,
        WRITE_REGISTER              = 0x16,
        READ_MULTIPLE_REGISTERS     = 0x23,
        GET_THE_TOKEN               = 0xFE,
        TOKEN_ACCEPTED              = 0xFF
    };

    enum NODE_STATES {
        SLAVE_IDLE                  = 0x00,
        SLAVE_HANDLING_MSG          = 0x01,
        SLAVE_ANSWERING             = 0x02,
        MASTER_IDLE                 = 0x03,
        MASTER_PREPARING_BROADCAST  = 0x04,
        MASTER_WAITING_TURNAROUND   = 0x05,
        MASTER_PREPARING_UNICAST    = 0x06,
        MASTER_WAITING_ANSWER       = 0x07,
        MASTER_HANDLING_MSG         = 0x08
    };

private:
    /* Node Control */
    unsigned char  _myAddress;                                 // All Nodes Should have an Address (even the master)
    bool           _isMaster;                                  // True if this node is a master node
    unsigned char  _thisNodeState;                             // Node state control
    
    /* RS-485 Control */
    MODBUS_BAUDRATE _baudRate;                                 // Baud Rate State
    static bool _safe2Write;                                   // Flag for bus not busy (wait for >= 3.5 chars after a frame)
    User_Timer *_modbusSafeBus;                                // User timer to handle _safe2Write flag

    /* CRC Control */
    static const unsigned char _auchCRCHi[];                   // Vector of values for faster calculation of CRC
    static const unsigned char _auchCRCLo[];                   // Vector of values for faster calculation of CRC
    unsigned char              _CRC[2];                        // CRC Hi and CRC Lo of a CRC-16 (modbus)
    
    /* Frame content control */
    unsigned char        _outputFrame[256];                     // Outgoing frame vector of chars
    unsigned char        _inputFrame[256];                      // Incoming frame vector of chars
    unsigned long        _incomingByteCounter;                  // Outgoing frame length
    unsigned long        _outputByteCounter;                    // Incoming frame length

    void _sendFrame()                                           // Write the _outputFrame on the bus
    {
        //Wait for >= 3.5 Chars
        while( !_safe2Write ){}
        //Write Bits on The Bus
        write( _outputFrame , _outputByteCounter );
        //Flag for the next waiting time
        _safe2Write = false;
        _modbusSafeBus->enable();
    }

    void _slaveRoutine()                                        // Routine of the Slave
    {
        //While a Slave
        while( !_isMaster )
        {
            //STATE SWITCH
            switch( _thisNodeState )
            {
                case SLAVE_IDLE:
                    //Wait for a Request
                    _waitForFrame();

                    //Check for a Valid Message
                    if( ( _incomingByteCounter >= 4 ) && (( _inputFrame[0] == 0 )||( _inputFrame[0] == _myAddress)) )
                    {
                        _thisNodeState = SLAVE_HANDLING_MSG;
                    }
                    else
                    {
                        _incomingByteCounter = 0;
                    }

                    if( _thisNodeState == SLAVE_IDLE)
                        break;

                case SLAVE_HANDLING_MSG:

                    switch( _inputFrame[1] )
                    {
                        case GET_THE_TOKEN:
                            //Request to be a master
                            if( _incomingByteCounter == 5 )
                            {
                                _outputFrame[0]    = _inputFrame[2];
                                _outputFrame[1]    = TOKEN_ACCEPTED;
                                _outputFrame[2]    = _myAddress;
                                _updateCRC(_outputFrame, 3);
                                _outputFrame[3]    = _CRC[0];
                                _outputFrame[4]    = _CRC[1];
                                _outputByteCounter = 5;
                                _thisNodeState     = SLAVE_ANSWERING;
                            }
                            break;
                        default:
                            //User application message
                            bool hasToAnswer = false;
                            requestHandler( _inputFrame, _incomingByteCounter, _outputFrame, &_outputByteCounter, &hasToAnswer);
                            if( hasToAnswer )
                            {
                                _updateCRC(_outputFrame, _outputByteCounter);
                                _outputFrame[++_outputByteCounter]  = _CRC[0];
                                _outputFrame[++_outputByteCounter]  = _CRC[1];
                                _thisNodeState                      = SLAVE_ANSWERING;
                            }
                    }    

                    _incomingByteCounter = 0;
                    if( _thisNodeState == SLAVE_HANDLING_MSG )
                    {
                        _thisNodeState = SLAVE_IDLE;
                        break;
                    }

                case SLAVE_ANSWERING:
                    //WRITING OUTPUT FRAME ON THE BUS
                    _sendFrame();

                    if( _outputFrame[1] == TOKEN_ACCEPTED )
                    {
                        _isMaster            = true;
                        _thisNodeState       = MASTER_IDLE;
                        _incomingByteCounter = 0;
                        return;
                    }
                    else
                    {
                        _thisNodeState       = SLAVE_IDLE;
                    }
                    break;
            }
        }
    }

    void _updateCRC(unsigned char* data, unsigned short dataLen) // Calculate the CRC-16 (modbus)
    {
        _CRC[0] = 0xFF; //Low-order Byte
        _CRC[1] = 0xFF; //High-order Byte
        unsigned char uIndex;
        for(unsigned char i=0; i<dataLen; i++)
        {
            uIndex  = _CRC[0] ^ data[i];
            _CRC[0] = _CRC[1] ^ _auchCRCHi[ uIndex ];
            _CRC[1] = _auchCRCLo[ uIndex ];
        }
    }
    
    long _waitForFrame()                                        // Wait for an input Frame (!! Should return -1 if timeout, but it was not implemented yet)
    {
        while( !isReadyToGet() ){}
        _incomingByteCounter = read( _inputFrame, 256 );
        while( 1 )
        {
            if( _baudRate == BAUDRATE_9600 )
            {
                Alarm::delay( 1145 );
            }
            else
            {
                Alarm::delay( 573 );
            }

            if( (_incomingByteCounter>=256) || (!isReadyToGet()) )
            {
                break;
            }
            else
            {
                _incomingByteCounter += read( &_inputFrame[_incomingByteCounter] , 256 - _incomingByteCounter );
            }
        }
        _safe2Write = false;
        _modbusSafeBus->enable();

        //!! Supposed to return -1 in case of timeout, but timeout handling and error frames were not implemented yet !!
        return _incomingByteCounter;
    }

public:
    modbusMultiMaster(unsigned char addr, bool startAsMaster, unsigned int baud_rate, unsigned int parity )
    : _rs485(0,19200,8,0,2)
    {

        _myAddress = addr;
        _CRC[0]    = 0;
        _CRC[1]    = 0;

        if( baud_rate == 19200 )
        {
            _baudRate = BAUDRATE_19200;
            _modbusSafeBus = new User_Timer( 0, 2000, &_safeBusHandler, false );
            _modbusSafeBus->disable();

            if(       parity == 0 ){ config( 19200, 8, 0, 2 ); }
            else if(  parity == 1 ){ config( 19200, 8, 1, 1 ); }
            else{                    config( 19200, 8, 2, 1 ); }
        }
        else
        {
            _baudRate = BAUDRATE_9600;
            _modbusSafeBus = new User_Timer( 0, 4000, &_safeBusHandler, false );
            _modbusSafeBus->disable();

            if(       parity == 0 ){ config( 9600, 8, 0, 2 ); }
            else if(  parity == 1 ){ config( 9600, 8, 1, 1 ); }
            else{                    config( 9600, 8, 2, 1 ); }
        }

        _isMaster  = startAsMaster;
        if( _isMaster )
        {
            _thisNodeState = MASTER_IDLE;
        }
        else
        {
            _thisNodeState = SLAVE_IDLE;
            _slaveRoutine();
        }
    }

    ~modbusMultiMaster()
    {
        _modbusSafeBus->disable();
        delete _modbusSafeBus;
    }

    /* Send/Request Methods */
    void giveTokenTo(                                           // Master passing the token to a Slave Node
                      unsigned char slaveAddress                //      Slave Address
                    )
    {
        if( !_isMaster ){ return; }

        _thisNodeState     = MASTER_PREPARING_UNICAST;
        _outputFrame[0]    = slaveAddress;                                                          // SLAVE ADDRESS
        _outputFrame[1]    = GET_THE_TOKEN;                                                    // FUNCTION CODE
        _outputFrame[2]    = _myAddress;                                                       // DATA = MASTER ADDRESS AS SLAVE
        _updateCRC(_outputFrame, 3);                                                           // CRC-16 (Modbus)
        _outputFrame[3]    = _CRC[0];
        _outputFrame[4]    = _CRC[1];
        _outputByteCounter = 5;
        _sendFrame();                                                                          // SENDING FRAME

        _thisNodeState     = MASTER_WAITING_ANSWER;                                            // WAITING ANSWER
        _waitForFrame();   
        
        _thisNodeState     = MASTER_HANDLING_MSG;                                              // HANDLING ANSWER
        if( ( _incomingByteCounter==5 ) &&
            ( _inputFrame[0] == _myAddress ) &&
            ( _inputFrame[1] == TOKEN_ACCEPTED  ) )
        {
            _isMaster = false;
            _thisNodeState = SLAVE_IDLE;
            _incomingByteCounter = 0;
            _slaveRoutine();
        }
    }

    void sendBroadcast(                                         // Master sending a broadcast message
                       unsigned char cmd,                       //      Function Code
                       char* data,                              //      Data Frame
                       unsigned int dataLen                     //      Data Frame Length
                      )
    {
        if( !_isMaster ){ return; }
        _thisNodeState          = MASTER_PREPARING_BROADCAST;
        _outputFrame[ 0 ]       = 0x00;                                                        // ADDRESS
        _outputFrame[ 1 ]       = cmd;                                                         // FUNCTION CODE
        unsigned char i         = 0;                                                           // DATA BYTES
        for(;i<dataLen;i++)
        {
            _outputFrame[i + 2] = data[i];
        }
        _updateCRC(_outputFrame, dataLen + 2);                                                 // CRC-16 (Modbus)
        _outputFrame[2 + i++]   = _CRC[0];
        _outputFrame[2 + i]     = _CRC[1];
        _outputByteCounter      = 4 + dataLen;
        _sendFrame();                                                                          // SENDING FRAME

        _thisNodeState = MASTER_WAITING_TURNAROUND;                                            // WAITING TURNAROUND
        Alarm::delay( 200 );                                                                   // DELAY 200us
        
        _thisNodeState = MASTER_IDLE;
    }
    
    void sendUnicast(                                           // Master sending a unicast message
                       unsigned char address,                   //      Slave Address
                       unsigned char cmd,                       //      Function Code
                       char* data,                              //      Data Frame
                       unsigned int dataLen,                    //      Data Frame Length
                       bool waitForAnswer                       //      Message Requires a Response
                    )
    {
        _thisNodeState        = MASTER_PREPARING_UNICAST;
        _outputFrame[0]       = address;                                                       // ADDRESS
        _outputFrame[1]       = cmd;                                                           // FUNCTION CODE
        unsigned long i       = 0;                                                             // DATA BYTES
        for(;i<dataLen;i++)
        {
            _outputFrame[2 + i] = data[i];
        }
        _updateCRC(_outputFrame, dataLen + 2);                                                 // CRC-16 (Modbus)
        _outputFrame[2 + i++] = _CRC[0];
        _outputFrame[2 + i]   = _CRC[1];
        _outputByteCounter    = 4 + dataLen;
        _sendFrame();                                                                          // SENDING FRAME
        
        if( waitForAnswer )
        {                                                                                      // WAITING ANSWER
            _thisNodeState = MASTER_WAITING_ANSWER;
            _waitForFrame();
            _thisNodeState = MASTER_HANDLING_MSG;                                              // HANDLING ANSWER
            responseHandler( _inputFrame, _incomingByteCounter );
        }    
        _thisNodeState = MASTER_IDLE;
    }

    /* User Control Methods */
    bool amIaMaster()                                           //  Returns _isMaster value
    { return _isMaster; }

    /* Receive Methods */
    virtual void requestHandler(                                //  When a Slave Node receive a Request Frame Message
                                  unsigned char *frame,         //      Received Frame Message
                                  unsigned long frameLen,       //      Received Frame Length
                                  unsigned char *outputAnswer,  //      Pointer for the Response Frame Message for the Request
                                  unsigned long *outputLen,     //      Pointer for the Response Frame Length (not include CRC!)
                                  bool *answer                  //      Pointer for the Flag to Answer (Write the Response Frame on the Bus or not)
                                );

    virtual void responseHandler(                               //  When a Master Node receive a Response Frame Message
                                  unsigned char *frame,         //      Received Frame Message
                                  unsigned long frameLen        //      Received Frame Length
                                );                      


    static void _safeBusHandler(const IC::Interrupt_Id &)     // Timer Handler to Control the _safe2Write flag
    {
        if( !_safe2Write ){ _safe2Write = true; }
    }
};




__END_SYS

#endif