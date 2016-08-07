/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech

Description: -

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#include "sx1276-hal.h"

const RadioRegisters_t SX1276MB1xAS::RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

SX1276MB1xAS::SX1276MB1xAS( RadioEvents_t *events,
                            PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                            PinName dio0, PinName dio1, PinName dio2, PinName dio3, /* PinName dio4, PinName dio5, */ PinName vswitch,
                            PinName antSwitch )
                            : SX1276( events, mosi, miso, sclk, nss, reset, dio0, dio1, dio2, dio3 /*, dio4, dio5 */,vswitch),
                            antSwitch( NC ),
                          //  antSwitch( NC ),
                      //  #if( defined ( TARGET_NUCLEO_L152RE ) )
                      //      fake( D8 )
                      //  #else
                      //      fake( A3 )
                     //   #endif
                            fake( NC )
{
    this->RadioEvents = events;

    Reset( );

    RxChainCalibration( );

    IoInit( );

    SetOpMode( RF_OPMODE_SLEEP );

    IoIrqInit( dioIrq );

    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = RF_IDLE ;
}

SX1276MB1xAS::SX1276MB1xAS( RadioEvents_t *events )
                        //#if defined ( TARGET_NUCLEO_L152RE )
                        //:   SX1276( events, D11 /*MOSI*/, D12 /*MISO*/, D13 /*SCLK*/, D10 /*CS*/, A0 /*RST*/, D2 /*DIO0*/, D3 /*DIO1*/, D4 /*DIO2*/, D5 /*DIO3*//*, A3, D9 */), // For NUCLEO L152RE dio4 is on port A3
                        :   SX1276( events, PB_5 /*MOSI*/, PB_4 /*MISO*/, PB_3 /*SCLK*/, PC_8 /*CS*/, PA_9 /*RST*/, PB_0 /*DIO0*/, PB_1 /*DIO1*/, PC_6 /*DIO2*/, PA_10 /*DIO3*//*, A3, D9 */ /*vswitch for inAir9b*/, PA_13), // For NUCLEO L152RE dio4 is on port A3
                        /*  antSwitch( A4 ),
                            fake( D8 )
                        */
                            antSwitch( antSwitch ), // automatic on inAir9b
                            fake( NC )
                        //#elif defined( TARGET_LPC11U6X )
                        //:   SX1276( events, D11, D12, D13, D10, A0, D2, D3, D4, D5, D8, D9 ),
                        //    antSwitch( P0_23 ), 
                        //    fake( A3 )
                     //   #else
                    //    :   SX1276( events, D11, D12, D13, D10, A0, D2, D3, D4, D5, D8, D9 ),
                     //       antSwitch( A4 ), 
                     //       fake( A3 )
                      //  #endif


{
    this->RadioEvents = events;

    Reset( );

    boardConnected = UNKNOWN;

    DetectBoardType( );

    RxChainCalibration( );

    IoInit( );

    SetOpMode( RF_OPMODE_SLEEP );
    IoIrqInit( dioIrq );

    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = RF_IDLE ;
}

//-------------------------------------------------------------------------
//                      Board relative functions
//-------------------------------------------------------------------------
uint8_t SX1276MB1xAS::DetectBoardType( void )
{
   /*
    if( boardConnected == UNKNOWN )
    {
        antSwitch.input( );
        wait_ms( 1 );
        if( antSwitch == 1 )
        {
            boardConnected = SX1276MB1LAS;
        }
        else
        {
            boardConnected = SX1276MB1MAS;
        }
        antSwitch.output( );
        wait_ms( 1 );
    }
    */
    boardConnected = SX1276MB1MAS;
    return ( boardConnected );
}

void SX1276MB1xAS::IoInit( void )
{
    AntSwInit( );
    SpiInit( );
    vswitch = 0; // voltage to antenna on inAir9b
}

void SX1276MB1xAS::RadioRegistersInit( )
{
    uint8_t i = 0;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SetModem( RadioRegsInit[i].Modem );
        Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }    
}

void SX1276MB1xAS::SpiInit( void )
{
    nss = 1;    
    spi.format( 8,0 );   
    uint32_t frequencyToSet = 8000000;
    //#if( defined ( TARGET_NUCLEO_L152RE ) ||  defined ( TARGET_LPC11U6X ) )
        spi.frequency( frequencyToSet );
    //#elif( defined ( TARGET_KL25Z ) ) //busclock frequency is halved -> double the spi frequency to compensate
    //    spi.frequency( frequencyToSet * 2 );
    //#else
    //    #warning "Check the board's SPI frequency"
    //#endif
    wait(0.1); 
}

void SX1276MB1xAS::IoIrqInit( DioIrqHandler *irqHandlers )
{
//#if( defined ( TARGET_NUCLEO_L152RE ) || defined ( TARGET_LPC11U6X ) )
    dio0.mode( PullDown );
    dio1.mode( PullDown );
    dio2.mode( PullDown );
    dio3.mode( PullDown );
   // dio4.mode( PullDown );
//#endif
    dio0.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[0] ) );
    dio1.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[1] ) );
    dio2.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[2] ) );
    dio3.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[3] ) );
   // dio4.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[4] ) );
}

void SX1276MB1xAS::IoReInit( void )
{
    //DigitalOut nss(PC_8,1);
    nss = DigitalOut (PC_8,1);
    vswitch = DigitalOut (PC_13, 0);
    reset = DigitalOut (PA_0, 0);
    spi = SPI(PB_5,PB_4,PB_3); //mosi, miso, sclk - re-initialise global object
    IoInit();
}

void SX1276MB1xAS::IoDeInit( void )
{
 
}

uint8_t SX1276MB1xAS::GetPaSelect( uint32_t channel )
{
    return RF_PACONFIG_PASELECT_PABOOST;
    /*
    if( channel > RF_MID_BAND_THRESH )
    {
        if( boardConnected == SX1276MB1LAS )
        {
            return RF_PACONFIG_PASELECT_PABOOST;
        }
        else
        {
            return RF_PACONFIG_PASELECT_RFO;
        }
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
    */
}

void SX1276MB1xAS::SetAntSwLowPower( bool status )
{
    if( isRadioActive != status )
    {
        isRadioActive = status;
    
        if( status == false )
        {
            AntSwInit( );
        }
        else
        {
            AntSwDeInit( );
        }
    }
}

void SX1276MB1xAS::AntSwInit( void )
{
     antSwitch = 0;
}

void SX1276MB1xAS::AntSwDeInit( void )
{
     antSwitch = 0;
}

void SX1276MB1xAS::SetAntSw( uint8_t rxTx )
{

    if (this->rxTx == rxTx)
        // no need to go further
        return;
    else    
        this->rxTx = rxTx;

    // 1: Tx, 0: Rx
    if( rxTx != 0 )
    {
        antSwitch = 1;
    }
    else
    {
        antSwitch = 0;
    }

}

bool SX1276MB1xAS::CheckRfFrequency( uint32_t frequency )
{
    //TODO: Implement check, currently all frequencies are supported
    return true;
}


void SX1276MB1xAS::Reset( void )
{
    reset.output();
    reset = 0;
    wait_ms( 1 );
    reset.input();
    wait_ms( 6 );
}
    
void SX1276MB1xAS::Write( uint8_t addr, uint8_t data )
{
    Write( addr, &data, 1 );
}

uint8_t SX1276MB1xAS::Read( uint8_t addr )
{
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}

void SX1276MB1xAS::Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    nss = 0;
    spi.write( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        spi.write( buffer[i] );
    }
    nss = 1;
}

void SX1276MB1xAS::Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    nss = 0;
    spi.write( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = spi.write( 0 );
    }
    nss = 1;
}

void SX1276MB1xAS::WriteFifo( uint8_t *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}

void SX1276MB1xAS::ReadFifo( uint8_t *buffer, uint8_t size )
{
    Read( 0, buffer, size );
}

