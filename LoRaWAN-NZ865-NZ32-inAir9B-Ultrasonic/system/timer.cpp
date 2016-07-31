/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer objects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"

Timer TimeCounter;
Ticker LoadTimeCounter;

volatile uint32_t CurrentTime = 0;

void TimerResetTimeCounter( void )
{
    CurrentTime = CurrentTime + TimeCounter.read_us( );
    TimeCounter.reset( );
    TimeCounter.start( );
}

void TimerTimeCounterInit( void )
{
    TimeCounter.start( );
    LoadTimeCounter.attach( &TimerResetTimeCounter, 10 );
}

TimerTime_t TimerGetCurrentTime( void )
{
    CurrentTime += TimeCounter.read_us( );
    TimeCounter.reset( );
    TimeCounter.start( );
    return ( ( TimerTime_t )CurrentTime );
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    CurrentTime += TimeCounter.read_us( );
    TimeCounter.reset( );
    TimeCounter.start( );
    return ( TimerTime_t )( CurrentTime - savedTime );
}

TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    CurrentTime += TimeCounter.read_us( );
    TimeCounter.reset( );
    TimeCounter.start( );
    return ( TimerTime_t )( CurrentTime + eventInFuture );
}

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->value = 0;
    obj->Callback = callback;
}

void TimerStart( TimerEvent_t *obj )
{
    obj->Timer.attach_us( obj->Callback, obj->value );
}

void TimerStop( TimerEvent_t *obj )
{
    obj->Timer.detach( );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    obj->value = value;
}
