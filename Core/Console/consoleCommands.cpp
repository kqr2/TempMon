// ConsoleCommands.c
// This is where you add commands:
//		1. Add a protoype
//			static eCommandResult_T ConsoleCommandVer(const char buffer[]);
//		2. Add the command to mConsoleCommandTable
//		    {"ver", &ConsoleCommandVer, HELP("Get the version string")},
//		3. Implement the function, using ConsoleReceiveParam<Type> to get the parameters from the buffer.

#include <string.h>
#include <stdio.h>
#include "consoleCommands.h"
#include "console.h"
#include "consoleIo.h"
#include "version.h"
#include "stm32f4xx_hal.h"
#include "sys.h"

extern sys_t sys;


#define IGNORE_UNUSED_VARIABLE(x)     if ( &x == &x ) {}

static eCommandResult_T ConsoleCommandComment(const char buffer[]);
static eCommandResult_T ConsoleCommandVer(const char buffer[]);
static eCommandResult_T ConsoleCommandHelp(const char buffer[]);

static eCommandResult_T ConsoleCommandRtcTime(const char buffer[]);
static eCommandResult_T ConsoleCommandRtcSet(const char buffer[]);
static eCommandResult_T ConsoleCommandSysSetInterval(const char buffer[]);


static const sConsoleCommandTable_T mConsoleCommandTable[] =
{
    {";", &ConsoleCommandComment, HELP("Comment! You do need a space after the semicolon. ")},
    {"help", &ConsoleCommandHelp, HELP("Lists the commands available")},
    {"ver", &ConsoleCommandVer, HELP("Get the version string")},

    {"rtc_time", &ConsoleCommandRtcTime, HELP("Get the current rtc time")},
    {"rtc_set", &ConsoleCommandRtcSet, HELP("Set the current rtc time")},
    {"sys_set_interval", &ConsoleCommandSysSetInterval, HELP("Set the system timer interval")},
    
    CONSOLE_COMMAND_TABLE_END // must be LAST
};

static eCommandResult_T ConsoleCommandComment(const char buffer[])
{
	// do nothing
	IGNORE_UNUSED_VARIABLE(buffer);
	return COMMAND_SUCCESS;
}

static eCommandResult_T ConsoleCommandHelp(const char buffer[])
{
	uint32_t i;
	uint32_t tableLength;
	eCommandResult_T result = COMMAND_SUCCESS;

    IGNORE_UNUSED_VARIABLE(buffer);

	tableLength = sizeof(mConsoleCommandTable) / sizeof(mConsoleCommandTable[0]);
	for ( i = 0u ; i < tableLength - 1u ; i++ )
	{
		ConsoleIoSendString(mConsoleCommandTable[i].name);
#if CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
		ConsoleIoSendString(" : ");
		ConsoleIoSendString(mConsoleCommandTable[i].help);
#endif // CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
		ConsoleIoSendString(STR_ENDLINE);
	}
	return result;
}

static eCommandResult_T ConsoleCommandVer(const char buffer[])
{
	eCommandResult_T result = COMMAND_SUCCESS;

    IGNORE_UNUSED_VARIABLE(buffer);

	ConsoleIoSendString(VERSION_STRING);
	ConsoleIoSendString(STR_ENDLINE);
	return result;
}


const sConsoleCommandTable_T* ConsoleCommandsGetTable(void)
{
	return (mConsoleCommandTable);
}


static eCommandResult_T ConsoleCommandRtcTime(const char buffer[])
{
  RV8803 *rtc = &sys.rtc;

  rtc->updateTime();
  printf("%s,%s\n\r", rtc->stringDateUSA(), rtc->stringTime());

  return COMMAND_SUCCESS;
}

static eCommandResult_T ConsoleCommandRtcSet(const char buffer[])
{
  char cmd[16];
  unsigned long unix;

  if (sscanf(buffer, "%s %lu", cmd, &unix) == 2) {
    RV8803 *rtc = &sys.rtc;
    rtc->setEpoch(unix, false);
  }

  return COMMAND_SUCCESS;
}

static eCommandResult_T ConsoleCommandSysSetInterval(const char buffer[])
{
  char cmd[20];
  int msec;

  if (sscanf(buffer, "%s %d", cmd, &msec) == 2) {
    sys_set_timer_interval(&sys, msec);
  }
  printf("Sys timer interval: %d\n\r", SYS_TIMER_INTERVAL(sys));

  return COMMAND_SUCCESS;
}

