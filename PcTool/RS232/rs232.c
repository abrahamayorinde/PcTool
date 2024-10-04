/*
***************************************************************************
*
* Author: Teunis van Beelen
*
* Copyright (C) 2005 - 2021 Teunis van Beelen
*
* Email: teuniz@protonmail.com
*
***************************************************************************
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***************************************************************************
*/


/* Last revision: February 9, 2021 */
/* For more info and how to use this library, visit: http://www.teuniz.net/RS-232/ */


#include "rs232.h"



#define RS232_PORTNR  38


int Cport[RS232_PORTNR],
    error;

static int Cport_fd;
struct termios new_port_settings,
       old_port_settings[RS232_PORTNR];

const char *comports[RS232_PORTNR]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3","/dev/ttyS4","/dev/ttyS5",
                                    "/dev/ttyS6","/dev/ttyS7","/dev/ttyS8","/dev/ttyS9","/dev/ttyS10","/dev/ttyS11",
                                    "/dev/ttyS12","/dev/ttyS13","/dev/ttyS14","/dev/ttyS15","/dev/ttyUSB0",
                                    "/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5",
                                    "/dev/ttyAMA0","/dev/ttyAMA1","/dev/ttyACM0","/dev/ttyACM1",
                                    "/dev/rfcomm0","/dev/rfcomm1","/dev/ircomm0","/dev/ircomm1",
                                    "/dev/cuau0","/dev/cuau1","/dev/cuau2","/dev/cuau3",
                                    "/dev/cuaU0","/dev/cuaU1","/dev/cuaU2","/dev/cuaU3"};

const char* comport_by_name;

int RS232_OpenComport(const char* comport_by_name, int baudrate, const char *mode, int flowctrl)
{
  int baudr,
      status;

  switch(baudrate)
  {
    case      50 : baudr = 50;//B50
                   break;
    case      75 : baudr = 75;//B75
                   break;
    case     110 : baudr = 110;//B110
                   break;
    case     134 : baudr = 134;//B134
                   break;
    case     150 : baudr = 150;//B150;
                   break;
    case     200 : baudr = 200;//B200;
                   break;
    case     300 : baudr = 300;//B300;
                   break;
    case     600 : baudr = 600;//B600;
                   break;
    case    1200 : baudr = 1200;//B1200;
                   break;
    case    1800 : baudr = 1800;//B1800;
                   break;
    case    2400 : baudr = 2400;//B2400;
                   break;
    case    4800 : baudr = 4800;//B4800;
                   break;
    case    9600 : baudr = 9600;//B9600;
                   break;
    case   19200 : baudr = 19200;//B19200;
                   break;
    case   38400 : baudr = 38400;//B38400;
                   break;
    case   57600 : baudr = 57600;//B57600;
                   break;
    case  115200 : baudr = 115200;//B115200;
                   break;
    case  230400 : baudr = 230400;//B230400;
                   break;
    case  460800 : baudr = 460800;//B460800
                   break;
#if defined(__linux__)
    case  500000 : baudr = 50000;// B500000;
                   break;
    case  576000 : baudr = 76000;//B576000;
                   break;
    case  921600 : baudr = 921600;//B921600;
                   break;
    case 1000000 : baudr = 100000;//B1000000;
                   break;
    case 1152000 : baudr = 1152000;//B1152000;
                   break;
    case 1500000 : baudr = 1500000;//B1500000;
                   break;
    case 2000000 : baudr = 2000000;//B2000000;
                   break;
    case 2500000 : baudr = 2500000;//B2500000;
                   break;
    case 3000000 : baudr = 3000000;//B3000000;
                   break;
    case 3500000 : baudr = 3500000;//B3500000;
                   break;
    case 4000000 : baudr = 4000000;//B4000000;
                   break;
#endif
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }

  int cbits=CS8,
      cpar=0,
      ipar=IGNPAR,
      bstop=0;

  if(strlen(mode) != 3)
  {
    printf("invalid mode \"%s\"\n", mode);
    return(1);
  }

  switch(mode[0])
  {
    case '8': cbits = CS8;
              break;
    case '7': cbits = CS7;
              break;
    case '6': cbits = CS6;
              break;
    case '5': cbits = CS5;
              break;
    default : printf("invalid number of data-bits '%c'\n", mode[0]);
              return(1);
              break;
  }

  switch(mode[1])
  {
    case 'N':
    case 'n': cpar = 0;
              ipar = IGNPAR;
              break;
    case 'E':
    case 'e': cpar = PARENB;
              ipar = INPCK;
              break;
    case 'O':
    case 'o': cpar = (PARENB | PARODD);
              ipar = INPCK;
              break;
    default : printf("invalid parity '%c'\n", mode[1]);
              return(1);
              break;
  }

  switch(mode[2])
  {
    case '1': bstop = 0;
              break;
    case '2': bstop = CSTOPB;
              break;
    default : printf("invalid number of stop bits '%c'\n", mode[2]);
              return(1);
              break;
  }

/*
http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html

http://man7.org/linux/man-pages/man3/termios.3.html
*/


  printf("Comport name: %s\n\r", comport_by_name);
  Cport_fd = open(comport_by_name, O_RDWR | O_NOCTTY | O_SYNC); //Opening a comport by name returns a file descriptor index
  printf("Opened comport name/File descriptor: %d\n\r", Cport_fd);

  if(Cport_fd==-1)
  {
    perror("unable to open comport ");
    return(1);
  }

  /* lock access so that another process can't also use the port */
  if(flock(Cport_fd, LOCK_EX | LOCK_NB) != 0)
  {
    close(Cport_fd);
    perror("Another process has locked the comport.");
    return(1);
  }

  error = tcgetattr(Cport_fd, old_port_settings /*+ comport_number*/);

  if(error==-1)
  {
    close(Cport_fd);

    flock(Cport_fd, LOCK_UN);  /* free the port so that others can use it. */

    perror("unable to read portsettings ");
    return(1);
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

  new_port_settings.c_cflag = cbits | cpar | bstop | CLOCAL | CREAD;
  if(flowctrl)
  {
    new_port_settings.c_cflag |= CRTSCTS;
  }
  new_port_settings.c_iflag = ipar;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
  new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */

  cfsetispeed(&new_port_settings, baudr);
  cfsetospeed(&new_port_settings, baudr);

  error = tcsetattr(Cport_fd, TCSANOW, &new_port_settings);
  if(error==-1)
  {
    tcsetattr(Cport_fd, TCSANOW, old_port_settings /*+ comport_number*/);

    close(Cport_fd);

    flock(Cport_fd, LOCK_UN);  /* free the port so that others can use it. */

    perror("unable to adjust portsettings ");
    return(1);
  }

/* http://man7.org/linux/man-pages/man4/tty_ioctl.4.html */

  if(ioctl(Cport_fd, TIOCMGET, &status) == -1)
  {
    tcsetattr(Cport_fd, TCSANOW, old_port_settings /*+ comport_number*/);

    flock(Cport_fd, LOCK_UN);  /* free the port so that others can use it. */
    perror("unable to get portstatus");
    return(1);
  }

  status |= TIOCM_DTR;    /* turn on DTR */
  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport_fd, TIOCMSET, &status) == -1)
  {
    tcsetattr(Cport_fd, TCSANOW, old_port_settings /*+ comport_number*/);

    flock(Cport_fd, LOCK_UN);  /* free the port so that others can use it. */

    perror("unable to set portstatus");
    return(1);
  }

  return(0);
}

int RS232_PollComport(int comport, unsigned char *buf, int size)
{
  int n;
  n = read(Cport_fd, buf, size);

  if(n < 0)
  {
    if(errno == EAGAIN)  return 0;
  }

  return(n);
}


int RS232_SendByte(int comport, unsigned char byte)
{
  int n = write(Cport_fd, &byte, 1);

  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return 0;
    }
    else
    {
      return 1;
    }
  }

  return(0);
}


int RS232_SendBuf(int comport, unsigned char *buf, int size)
{
  int n = write(Cport_fd, buf, size);

  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return 0;
    }
    else
    {
      return -1;
    }
  }

  return(n);
}


void RS232_CloseComport(int comport)
{
  int status;

  if(ioctl(Cport_fd, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */
  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport_fd, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }

  tcsetattr(Cport_fd, TCSANOW, old_port_settings /*+ comport_number*/);

  close(Cport_fd);

  flock(Cport_fd, LOCK_UN);  /* free the port so that others can use it. */
}

/*
Constant  Description
TIOCM_LE        DSR (data set ready/line enable)
TIOCM_DTR       DTR (data terminal ready)
TIOCM_RTS       RTS (request to send)
TIOCM_ST        Secondary TXD (transmit)
TIOCM_SR        Secondary RXD (receive)
TIOCM_CTS       CTS (clear to send)
TIOCM_CAR       DCD (data carrier detect)
TIOCM_CD        see TIOCM_CAR
TIOCM_RNG       RNG (ring)
TIOCM_RI        see TIOCM_RNG
TIOCM_DSR       DSR (data set ready)

http://man7.org/linux/man-pages/man4/tty_ioctl.4.html
*/

int RS232_IsDCDEnabled(int comport)
{
  int status;

  ioctl(Cport_fd, TIOCMGET, &status);

  if(status&TIOCM_CAR) return(1);
  else return(0);
}


int RS232_IsRINGEnabled(int comport)
{
  int status;

  ioctl(Cport_fd, TIOCMGET, &status);

  if(status&TIOCM_RNG) return(1);
  else return(0);
}


int RS232_IsCTSEnabled(int comport)
{
  int status;

  ioctl(Cport_fd, TIOCMGET, &status);


  if(status&TIOCM_CTS) return(1);
  else return(0);
}


int RS232_IsDSREnabled(int comport)
{
  int status;

  ioctl(Cport_fd, TIOCMGET, &status);


  if(status&TIOCM_DSR) return(1);
  else return(0);
}


void RS232_enableDTR(int comport)
{
  int status;

  if(ioctl(Cport_fd, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_DTR;    /* turn on DTR */

  if(ioctl(Cport_fd, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}


void RS232_disableDTR(int comport)
{
  int status;

  if(ioctl(Cport_fd, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */

  if(ioctl(Cport_fd, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}


void RS232_enableRTS(int comport)
{
  int status;

  if(ioctl(Cport_fd, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport_fd, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}


void RS232_disableRTS(int comport)
{
  int status;

  if(ioctl(Cport_fd, TIOCMGET, &status) == -1)  
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport_fd, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}


void RS232_flushRX(int comport)
{
  tcflush(Cport_fd, TCIFLUSH);
}


void RS232_flushTX(int comport)
{
  tcflush(Cport_fd, TCOFLUSH);
}


void RS232_flushRXTX(int comport)
{
  tcflush(Cport_fd, TCIOFLUSH);
}





void RS232_cputs(int comport_number, const char *text)  /* sends a string to serial port */
{
  while(*text != 0)   RS232_SendByte(comport_number, *(text++));
}


/* return index in comports matching to device name or -1 if not found */
int RS232_GetPortnr(const char *devname)
{
  int i;

  char str[32];

  strcpy(str, "/dev/");

  strncat(str, devname, 16);
  str[31] = 0;

  for(i=0; i<RS232_PORTNR; i++)
  {
    if(!strcmp(comports[i], str))
    {
      return i;
    }
  }

  return -1;  /* device not found */
}











