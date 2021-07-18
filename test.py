import serial

global serialport
global data_list
global tmp_value
global data_count
global error_count
data_count = 0
error_count = 0

def init():
    global serialport
    global data_list
    global tmp_value
    serialport = serial.Serial ("/dev/ttyS0")    #Open named port
    serialport.baudrate = 115200
    data_list = []
    tmp_value = ""
    

def data_handler(data):
    global data_count
    data_count = data_count +1
    print(error_count,data_count,data)
    
init()
while(True):
    try:
        read_byte = serialport.read()
        if(read_byte.decode() == '\n'):
            data_list.append(float(tmp_value))
            tmp_value = ""
            data_handler(data_list)
            data_list = []
        elif(read_byte.decode() == ' '):
            data_list.append(float(tmp_value))
            tmp_value = ""
        else:
            tmp_value = tmp_value + (read_byte.decode())
            
    except Exception as e:
        global error_count
        error_count = error_count +1
        print(error_count,e)
        init()
