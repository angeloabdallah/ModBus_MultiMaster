#include <alarm.h>
#include <modbusMultiMaster.h>

/*
    Purpose:    Testing the Modified Version of ModBus RTU Wich Supports Multi-Master (One at a time)
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
*/

using namespace EPOS;
OStream cout; 

//Constants of Project
const bool startAsMaster         = false;       // True for a Master Node, False for a Slave Node
const char thisNodeAddress       = 0x01;        // Slave Address
const char nextMasterAddress     = 0x02;        // When passing the token to another node
const unsigned int baud_rate     = 9600;        // Baud Rate for RS485 ModBus RTU
const unsigned int parity        = 2;           // 0 = None, 1 = Odd, 2 = Even
const char howManyOthers         = 2;           // How many other nodes on the bus
const char otherNodeAddresses[]  = {            // Other nodes addresses on the bus
                                    0x02,
                                    0x03
                                   };

int main()
{
    //Wait before starting, make sure every node is on the bus.
    Alarm::delay( 10e6 );

    //In order to help debugging information with cutecom or similar
    cout << "\n\nHello, I am Node 0x" << static_cast<int> (thisNodeAddress) << " and I am a ";
    if(startAsMaster)
        cout << "Master" << endl;
    else
        cout << "Slave" << endl;

    //Setup
    modbusMultiMaster thisNodeRTU( 
                                    thisNodeAddress,
                                    startAsMaster,
                                    baud_rate,
                                    parity
                                 );

    //Loop
    while(1)
    {
        //Sending a Broadcast Message
        char msg[256] = {'N','o','d','e',' ','x',' ','s','a','y','s',' ','h','i','!',0};
        unsigned long msgLen   = 15;
        msg[5]                 = thisNodeAddress + 48;
        cout << "\nSending Broadcast Message \"" << msg << "\"" << endl;
        thisNodeRTU.sendBroadcast( 0x16, msg, msgLen );
        
        //Sending Unicast Messages (Request) for Each Slave on the Bus and waiting for their Response Frame
        for( unsigned int i=0; i<howManyOthers; i++ )
        {
            cout << "\nSending Unicast Message \"" << msg << "\" to Node " << static_cast<int> (otherNodeAddresses[i]);
            thisNodeRTU.sendUnicast(
                                    otherNodeAddresses[ i ],
                                    0x16,
                                    msg,
                                    msgLen,
                                    true
                                   );
        }

        //Passing the Token
        cout << "\nPassing the token to the Node 0x" << static_cast<int> (nextMasterAddress) << endl;
        thisNodeRTU.giveTokenTo( nextMasterAddress );
    }
    
    return 0;
}

/* Receive Methods */
void modbusMultiMaster::requestHandler(                               //  When a Slave Node receive a Request Frame Message
                                            unsigned char *frame,         //      Received Frame Message
                                            unsigned long frameLen,       //      Received Frame Length
                                            unsigned char *outputAnswer,  //      Pointer for the Response Frame Message for the Request
                                            unsigned long *outputLen,     //      Pointer for the Response Frame Length (not include CRC!)
                                            bool *answer                  //      Pointer for the Flag to Answer (Write the Response Frame on the Bus or not)
                                        )
{
    //Printing the whole frame received, including CRC
    cout << "\n[ This is the Request Handler ]" << endl;
    cout << "\t{ ";
    unsigned long i = 0;
    for(;i<frameLen-1;i++)
    {
        cout << frame[i] << " , ";
    }
    cout << frame[i] << " } " << endl;

    //Handling the data content (Function Codes were not implemented for this demo!)
    cout << "\tReceived a ";
    if( frame[0] == 0x00 ){ cout << "Broadcast"; }
    else{                   cout << "Unicast";   }
    cout << " Message with the Function Code \"" << static_cast<int> (frame[1]) << "\"" << endl;
    if( frameLen > 4 )
    {
        cout << " Message content: {";
        for(i=2; i<frameLen-2;i++)
        {
            cout << frame[i];
        }
    }
    cout << "}" << endl;

    //Answer only if it was a unicast message
    if( frame[0] != 0x00 )
    {
        cout << "Answering with the message content \"ok\"" << endl;
        *answer = true;
        outputAnswer[0] = frame[0];
        outputAnswer[1] = frame[1];
        outputAnswer[2] = 'o';
        outputAnswer[3] = 'k';
        *outputLen      = 4;
    }
}

void modbusMultiMaster::responseHandler(                              //  When a Master Node receive a Response Frame Message
                                        unsigned char *frame,         //      Received Frame Message
                                        unsigned long frameLen        //      Received Frame Length
                                        )
{
    //Printing the whole frame received, including CRC
    cout << "\n[ This is the Response Handler ]" << endl;
    cout << "\t{ ";
    unsigned long i = 0;
    for(;i<frameLen-1;i++)
    {
        cout << frame[i] << " , ";
    }
    cout << frame[i] << " } " << endl;

    //Handling the data content (Function Codes were not implemented for this demo!)
    cout << "\tReceived a Response Message with the Function Code \"" << static_cast<int> (frame[1]) << "\"" << endl;
    if( frameLen > 4 )
    {
        cout << " Message content: {";
        for(i=2; i<frameLen-2;i++)
        {
            cout << frame[i];
        }
    }
    cout << "}" << endl;

}