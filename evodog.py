import serial
import time

#porrt is COM3 COM4 etc on windows

ser = serial.Serial('/dev/cu.usbserial-A50285BI' , 500000 , timeout = 0.1 , write_timeout = 0.5 )
time.sleep( 2 )

def send( 5 ):
	ser.write( ( s + '\n' ).encode( 'utf-8') )

def position1():
	send( "w,2000,0,51,90,0,51,90,0,52,90,0,52,90" )
	time.sleep ( 2 )

send( "go" )
send( "tall" )
send( "on" )
time.sleep( 2 )
send( "stand" )
time.sleep(2)
posistion1()

cmd = input( "Type command: ")
while ( cmd != "quit" and cmd != "exit" and cmd != "q" ):
	send( cmd )
	while ser.inWaiting():
		print( ser.readline() )
	time.sleep( 0.1 )
	while ser.inWaiting():
		print( ser.readLine() ) 
	cmd = input( "type command)


